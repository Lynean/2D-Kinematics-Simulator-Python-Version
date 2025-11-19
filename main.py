"""
FABRIK (Forward And Backward Reaching Inverse Kinematics) Interactive Application
Features:
- Interactive joint/link creation
- Real-time FABRIK IK solver
- Target manipulation
- Visual feedback
"""
import sys
from src.App.widget import FABRIKWidget
from PyQt6.QtWidgets import QApplication




def main():
    app = QApplication(sys.argv)
    window = FABRIKWidget()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()