import numpy as np

def UAV_SetUp():
    UAV = {}
    UAV['S'] = np.array([2, 2, 15])  # Start position (x,y,z)
    UAV['G'] = np.array([18, 18, 8])  # Goal position (x,y,z)
    UAV['PointNum'] = 3  # Number of navigation points for the UAV

    UAV['PointDim'] = UAV['S'].shape[0]

    # Updated no-fly zones (x, y, height, radius)
    UAV['NoFlyZones'] = np.array([
        [5, 5, 12, 2],
        [15, 7, 10, 2],
        [10, 12, 14, 2.5],
        [8, 15, 16, 2.5],
        [12, 3, 11, 2],
        [3, 10, 15, 2],
        [7, 7, 18, 2],
        [13, 9, 12, 2],
        [16, 13, 10, 2],
    ])

    # UAV constraint settings
    UAV['limt'] = {
        'x': [0, 20],
        'y': [0, 20],
        'z': [0, 20],
    }

    # Ensure start and end positions are not inside any cylinder
    if not is_position_valid(UAV['S'], UAV['NoFlyZones']):
        raise ValueError("Start position is inside a no-fly zone!")
    if not is_position_valid(UAV['G'], UAV['NoFlyZones']):
        raise ValueError("Goal position is inside a no-fly zone!")

    return UAV

def is_position_valid(position, no_fly_zones):
    for zone in no_fly_zones:
        x, y, height, radius = zone
        dx = position[0] - x
        dy = position[1] - y
        distance_2d = np.sqrt(dx**2 + dy**2)
        if distance_2d <= radius and 0 <= position[2] <= height:
            return False
    return True