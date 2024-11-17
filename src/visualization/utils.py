import numpy as np
from tqdm import tqdm
import json
from ..utils.encoders import NumpyEncoder
from .config import COLORS

def load_animation_data(filename='animation_export.json'):
    """Load animation data from JSON file."""
    with open(filename, 'r') as f:
        data = json.load(f)
    
    # Convert lists back to numpy arrays where necessary
    data['UAV']['S'] = np.array(data['UAV']['S'])
    data['UAV']['G'] = np.array(data['UAV']['G'])
    data['UAV']['NoFlyZones'] = np.array(data['UAV']['NoFlyZones'])
    data['Fitness_list'] = np.array(data['Fitness_list'])
    
    return data

def interpolate_paths(paths, frames_per_iteration):
    """Interpolate between path points for smooth animation."""
    interpolated_paths = []
    total_frames = (len(paths) - 1) * frames_per_iteration
    
    with tqdm(total=total_frames, desc="Interpolating paths") as pbar:
        for i in range(len(paths) - 1):
            for j in range(frames_per_iteration):
                t = j / frames_per_iteration
                frame = []
                for wolf_path_start, wolf_path_end in zip(paths[i], paths[i+1]):
                    interpolated_wolf_path = []
                    for point_start, point_end in zip(wolf_path_start, wolf_path_end):
                        interpolated_point = [
                            (1-t) * point_start[k] + t * point_end[k] for k in range(3)
                        ]
                        interpolated_wolf_path.append(interpolated_point)
                    frame.append(interpolated_wolf_path)
                interpolated_paths.append(frame)
                pbar.update(1)
    
    return interpolated_paths

def create_cylinder(center_x, center_y, height, radius, resolution=20):
    """Create cylinder coordinates for visualization."""
    theta = np.linspace(0, 2*np.pi, resolution)
    z = np.linspace(0, height, 2)
    theta, z = np.meshgrid(theta, z)
    
    x = radius * np.cos(theta) + center_x
    y = radius * np.sin(theta) + center_y
    
    # Create top circle
    theta_top = np.linspace(0, 2*np.pi, resolution)
    x_top = radius * np.cos(theta_top) + center_x
    y_top = radius * np.sin(theta_top) + center_y
    z_top = np.full_like(x_top, height)
    
    return x, y, z, x_top, y_top, z_top

def precompute_animation_data(all_paths, UAV, obj_fun, fps, duration):
    """Precompute all animation data for smoother rendering."""
    frames_per_iteration = int(fps * duration / len(all_paths))
    smooth_paths = interpolate_paths(all_paths, frames_per_iteration)
    
    precomputed_data = []
    
    print("Precomputing animation data...")
    for frame, current_paths in enumerate(tqdm(smooth_paths)):
        fitnesses = [obj_fun(np.array(path).flatten(), UAV) for path in current_paths]
        min_fitness = min(fitnesses)
        max_fitness = max(fitnesses)
        norm_fitnesses = (fitnesses - min_fitness) / (max_fitness - min_fitness + 1e-10)
        
        frame_data = []
        for path, norm_fitness in zip(current_paths, norm_fitnesses):
            x = [UAV['S'][0]] + [p[0] for p in path] + [UAV['G'][0]]
            y = [UAV['S'][1]] + [p[1] for p in path] + [UAV['G'][1]]
            z = [UAV['S'][2]] + [p[2] for p in path] + [UAV['G'][2]]
            
            color = get_path_color(norm_fitness)
            
            frame_data.append({
                'path': (x, y, z),
                'color': color,
                'nav_points': (
                    [p[0] for p in path],
                    [p[1] for p in path],
                    [p[2] for p in path]
                ),
                'normalized_fitness': norm_fitness
            })
        
        precomputed_data.append({
            'frame_data': frame_data,
            'best_fitness': min_fitness,
            'iteration': frame // frames_per_iteration
        })
    
    return precomputed_data

def get_path_color(quality):
    """Get color based on path quality."""
    if quality < 0.33:
        return COLORS['secondary']['red']
    elif quality < 0.67:
        return COLORS['secondary']['yellow']
    else:
        return COLORS['secondary']['green']