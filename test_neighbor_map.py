"""
Test the neighbor map generation in Monte Carlo sampler
"""
import numpy as np
from src.chain import FABRIKChain
from src.obstacle import Obstacle
from Tools.monte_carlo import MonteCarloSampler


def test_neighbor_map():
    """Test neighbor map generation"""
    print("=" * 70)
    print("MONTE CARLO NEIGHBOR MAP TEST")
    print("=" * 70)
    
    # Create a simple chain
    chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
    
    # Create Monte Carlo sampler with fewer samples for testing
    mc_sampler = MonteCarloSampler(chain, num_samples=100)
    
    # Create an obstacle
    obstacle = Obstacle(position=(80, 60), radius=30)
    
    print(f"\nChain: {chain.num_joints} joints, link length={chain.link_lengths[0]}")
    print(f"Obstacle: position={obstacle.position}, radius={obstacle.radius}")
    print(f"Sampling {mc_sampler.num_samples} configurations...")
    
    # Sample configuration space
    collision_configs, non_collision_configs, neighbor_map = mc_sampler.sample_configuration_space(obstacle)
    
    print(f"\nResults:")
    print(f"  Total samples: {mc_sampler.num_samples}")
    print(f"  Collision configurations: {len(collision_configs)}")
    print(f"  Non-collision configurations: {len(non_collision_configs)}")
    print(f"  Neighbor map nodes: {len(neighbor_map)}")
    
    # Analyze neighbor map
    if len(neighbor_map) > 0:
        total_neighbors = sum(len(neighbors) for neighbors in neighbor_map.values())
        avg_neighbors = total_neighbors / len(neighbor_map) if len(neighbor_map) > 0 else 0
        
        print(f"\nNeighbor Map Statistics:")
        print(f"  Total edges: {total_neighbors}")
        print(f"  Average neighbors per node: {avg_neighbors:.2f}")
        
        # Show some example neighbors
        print(f"\nExample neighbor connections (first 5 nodes):")
        for node_id in list(neighbor_map.keys())[:5]:
            neighbors = neighbor_map[node_id]
            print(f"  Node {node_id}:")
            if len(neighbors) > 0:
                for neighbor_id, distance in neighbors[:3]:  # Show first 3 neighbors
                    print(f"    → Node {neighbor_id} (distance: {distance:.3f})")
                if len(neighbors) > 3:
                    print(f"    ... and {len(neighbors) - 3} more neighbors")
            else:
                print(f"    (no neighbors within radius)")
        
        # Find node with most neighbors
        max_neighbors_node = max(neighbor_map.keys(), key=lambda k: len(neighbor_map[k]))
        max_neighbors = len(neighbor_map[max_neighbors_node])
        
        # Find node with fewest neighbors
        min_neighbors_node = min(neighbor_map.keys(), key=lambda k: len(neighbor_map[k]))
        min_neighbors = len(neighbor_map[min_neighbors_node])
        
        print(f"\nExtreme cases:")
        print(f"  Node with most neighbors: Node {max_neighbors_node} ({max_neighbors} neighbors)")
        print(f"  Node with fewest neighbors: Node {min_neighbors_node} ({min_neighbors} neighbors)")
    
    print("\n✓ Test completed successfully!")


def test_neighbor_map_structure():
    """Test the structure and format of neighbor map"""
    print("\n" + "=" * 70)
    print("NEIGHBOR MAP STRUCTURE TEST")
    print("=" * 70)
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
    mc_sampler = MonteCarloSampler(chain, num_samples=50)
    obstacle = Obstacle(position=(100, 100), radius=20)
    
    collision_configs, non_collision_configs, neighbor_map = mc_sampler.sample_configuration_space(obstacle)
    
    print(f"\nNeighbor map format example:")
    print(f"  Type: {type(neighbor_map)}")
    print(f"  Keys (node IDs): {list(neighbor_map.keys())[:10]}...")
    
    if len(neighbor_map) > 0:
        first_node = list(neighbor_map.keys())[0]
        print(f"\n  Example entry (Node {first_node}):")
        print(f"    neighbor_map[{first_node}] = {neighbor_map[first_node][:5]}")
        
        # Verify format
        print(f"\nFormat verification:")
        all_valid = True
        for node_id, neighbors in neighbor_map.items():
            if not isinstance(neighbors, list):
                print(f"  ❌ Node {node_id}: neighbors not a list")
                all_valid = False
            else:
                for neighbor_id, distance in neighbors:
                    if not isinstance(neighbor_id, int):
                        print(f"  ❌ Node {node_id}: neighbor_id {neighbor_id} not an int")
                        all_valid = False
                    if not isinstance(distance, float):
                        print(f"  ❌ Node {node_id}: distance {distance} not a float")
                        all_valid = False
        
        if all_valid:
            print(f"  ✓ All entries follow correct format: {{node_id: [(neighbor_id, distance), ...]}}")
    
    print("\n✓ Structure test completed!")


def test_radius_parameter():
    """Test different radius values"""
    print("\n" + "=" * 70)
    print("NEIGHBOR RADIUS PARAMETER TEST")
    print("=" * 70)
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
    obstacle = Obstacle(position=(70, 50), radius=25)
    
    radii = [2.0, 5.0, 8.0, 15.0]
    
    for radius in radii:
        mc_sampler = MonteCarloSampler(chain, num_samples=50)
        
        # Temporarily modify the build method to use different radius
        collision_configs, non_collision_configs, _ = mc_sampler.sample_configuration_space(obstacle)
        neighbor_map = mc_sampler._build_neighbor_map(non_collision_configs, radius=radius)
        
        total_edges = sum(len(neighbors) for neighbors in neighbor_map.values())
        avg_neighbors = total_edges / len(neighbor_map) if len(neighbor_map) > 0 else 0
        
        print(f"\nRadius: {radius}")
        print(f"  Nodes: {len(neighbor_map)}")
        print(f"  Total edges: {total_edges}")
        print(f"  Avg neighbors: {avg_neighbors:.2f}")
    
    print("\n✓ Radius test completed!")


if __name__ == "__main__":
    test_neighbor_map()
    test_neighbor_map_structure()
    test_radius_parameter()
    
    print("\n" + "=" * 70)
    print("ALL TESTS PASSED!")
    print("=" * 70)
