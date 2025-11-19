"""
Serial transmitter for sending joint angles to Arduino
"""
import serial
import serial.tools.list_ports
import numpy as np
import time
from typing import Optional, List


class SerialTransmitter:
    """
    Transmits joint angle data to Arduino via serial communication
    """
    
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
        """
        Initialize serial transmitter
        
        Args:
            port: Serial port name (e.g., 'COM3', '/dev/ttyUSB0'). Auto-detect if None.
            baudrate: Communication speed (default: 115200)
        """
        self.baudrate = baudrate
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False
        self.port_name = port
        
        if port:
            self.connect(port)
    
    @staticmethod
    def list_available_ports():
        """
        List all available serial ports
        
        Returns:
            List of tuples (port_name, description)
        """
        ports = serial.tools.list_ports.comports()
        return [(port.device, port.description) for port in ports]
    
    def connect(self, port: str) -> bool:
        """
        Connect to specified serial port
        
        Args:
            port: Serial port name
            
        Returns:
            True if connection successful
        """
        try:
            if self.is_connected:
                self.disconnect()
            
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=1
            )
            
            # Wait for Arduino to reset
            time.sleep(2)
            
            # Clear input buffer
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.is_connected = True
            self.port_name = port
            print(f"Connected to {port} at {self.baudrate} baud")
            return True
            
        except serial.SerialException as e:
            print(f"Failed to connect to {port}: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        if self.serial_port and self.is_connected:
            try:
                self.serial_port.close()
                print(f"Disconnected from {self.port_name}")
            except Exception as e:
                print(f"Error disconnecting: {e}")
        
        self.is_connected = False
        self.serial_port = None
    
    def send_angles(self, angles: np.ndarray, format: str = 'degrees') -> bool:
        """
        Send joint angles to Arduino
        
        Args:
            angles: Array of joint angles (relative angles)
            format: 'degrees' or 'radians' - format to send to Arduino
            
        Returns:
            True if send successful
        """
        if not self.is_connected or self.serial_port is None:
            print("Not connected to serial port")
            return False
        
        try:
            # Convert to degrees if needed
            if format == 'degrees':
                angles_to_send = np.degrees(angles)
            else:
                angles_to_send = angles
            
            # Format: "A:<angle0>,<angle1>,<angle2>,...\n"
            # Example: "A:45.5,90.0,-30.2\n"
            angle_str = ','.join([f"{angle:.2f}" for angle in angles_to_send])
            message = f"A:{angle_str}\n"
            
            # Send message
            self.serial_port.write(message.encode('utf-8'))
            self.serial_port.flush()
            
            # Print angles being sent
            print(f"Sent angles (degrees): {[f'{a:.1f}°' for a in angles_to_send]}")
            print(f"Raw message sent: {repr(message)}")
            
            # Try to read response
            time.sleep(0.05)  # Give Arduino time to respond
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.readline().decode('utf-8').strip()
                print(f"Arduino response: {response}")
            
            return True
            
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            self.is_connected = False
            return False
        except Exception as e:
            print(f"Unexpected error: {e}")
            return False
    
    def send_raw(self, message: str) -> bool:
        """
        Send raw string message to Arduino
        
        Args:
            message: String message to send
            
        Returns:
            True if send successful
        """
        if not self.is_connected or self.serial_port is None:
            print("Not connected to serial port")
            return False
        
        try:
            if not message.endswith('\n'):
                message += '\n'
            
            self.serial_port.write(message.encode('utf-8'))
            self.serial_port.flush()
            return True
            
        except Exception as e:
            print(f"Error sending raw message: {e}")
            return False
    
    def read_response(self, timeout: float = 0.5) -> Optional[str]:
        """
        Read response from Arduino
        
        Args:
            timeout: Time to wait for response (seconds)
            
        Returns:
            Response string or None if no response
        """
        if not self.is_connected or self.serial_port is None:
            return None
        
        try:
            old_timeout = self.serial_port.timeout
            self.serial_port.timeout = timeout
            
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.readline().decode('utf-8').strip()
                self.serial_port.timeout = old_timeout
                return response
            
            self.serial_port.timeout = old_timeout
            return None
            
        except Exception as e:
            print(f"Error reading response: {e}")
            return None
    
    def __del__(self):
        """Cleanup on deletion"""
        self.disconnect()


class ChainSerialBridge:
    """
    Bridge between FABRIKChain and SerialTransmitter
    Automatically sends angle updates when chain moves
    """
    
    def __init__(self, chain, transmitter: SerialTransmitter):
        """
        Initialize bridge
        
        Args:
            chain: FABRIKChain instance
            transmitter: SerialTransmitter instance
        """
        self.chain = chain
        self.transmitter = transmitter
        self.auto_send = False
        self.send_interval = 0.05  # Send every 50ms (20Hz)
        self.last_send_time = 0
        self.last_angles = None
    
    def enable_auto_send(self, enabled: bool = True):
        """
        Enable/disable automatic sending of angle updates
        
        Args:
            enabled: True to enable auto-send
        """
        self.auto_send = enabled
        if enabled:
            print("Auto-send enabled - angles will be sent on update")
        else:
            print("Auto-send disabled")
    
    def update(self):
        """
        Call this in the main update loop to send angles if needed
        """
        if not self.auto_send or not self.transmitter.is_connected:
            return
        
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_send_time < self.send_interval:
            return
        
        # Get current angles
        current_angles = self.chain.get_joint_angles()
        
        # Check if angles changed (avoid sending duplicate data)
        if self.last_angles is not None and np.allclose(current_angles, self.last_angles, atol=0.001):
            return
        
        # Send angles
        if self.transmitter.send_angles(current_angles, format='degrees'):
            self.last_angles = current_angles.copy()
            self.last_send_time = current_time
    
    def send_now(self):
        """Force send current angles immediately"""
        if self.transmitter.is_connected:
            current_angles = self.chain.get_joint_angles()
            self.transmitter.send_angles(current_angles, format='degrees')
            self.last_angles = current_angles.copy()
