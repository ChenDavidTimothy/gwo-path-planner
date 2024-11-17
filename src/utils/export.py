from .encoders import NumpyEncoder
import json

def export_animation_data(solution, UAV, filename='animation_export.json'):
    """Export animation data to JSON file."""
    export_data = {
        'all_paths': solution['all_paths'],
        'UAV': {
            'S': UAV['S'],
            'G': UAV['G'],
            'NoFlyZones': UAV['NoFlyZones'],
            'limt': UAV['limt'],
            'PointNum': UAV['PointNum'],
            'PointDim': UAV['PointDim']
        },
        'Fitness_list': solution['Fitness_list'],
        'seed': solution['seed']
    }
    
    with open(filename, 'w') as f:
        json.dump(export_data, f, cls=NumpyEncoder)