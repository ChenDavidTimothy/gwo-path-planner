import numpy as np
import random
from src.core.gwo import GWO
from src.core.uav_setup import UAV_SetUp
from src.core.obj_fun import ObjFun
from src.utils.export import export_animation_data
from src.visualization.animation import PathAnimator, save_animation

def set_seed(seed=None):
    """Set random seed for reproducibility."""
    if seed is None:
        seed = random.randint(0, 100000)
    np.random.seed(seed)
    random.seed(seed)
    return seed

def main():
    # Algorithm parameters
    SearchAgents = 30
    Max_iter = 200

    # Setup
    UAV = UAV_SetUp()
    seed = set_seed(None)  # Change to an integer for a fixed seed
    print(f"Using seed: {seed}")

    # Run optimization
    solution = GWO(UAV, SearchAgents, Max_iter, seed)
    
    # Export data
    export_animation_data(solution, UAV)
    
    # Create and save animation
    animator = PathAnimator(UAV, ObjFun)
    animation, total_frames = animator.create_animation(solution['all_paths'])
    save_animation(animation, total_frames)
    
    print(f"Optimization completed and animation saved. Seed used: {seed}")

if __name__ == "__main__":
    main()