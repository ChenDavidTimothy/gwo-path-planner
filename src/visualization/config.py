# Color palette
COLORS = {
    'main': {
        'background': '#252426',
        'primary1': '#F0A050',
        'primary2': '#C07830',
        'primary3': '#E0C0A0',
        'primary4': '#805030'
    },
    'secondary': {
        'red': '#E04040',
        'green': '#50B050',
        'yellow': '#F0C040',
        'blue': '#4080C0',
        'purple': '#8040A0'
    }
}

# Animation settings
DPI = 100
FPS = 30
DURATION = 10  # Duration in seconds
ROTATION_ANGLE = 360  # Two full rotations

def setup_plot_style(ax, fig, UAV):
    """Configure the 3D plot style."""
    import matplotlib.pyplot as plt
    import numpy as np
    
    plt.style.use('dark_background')
    
    # Set background color
    ax.set_facecolor(COLORS['main']['background'])
    fig.patch.set_facecolor(COLORS['main']['background'])

    # Adjust the plot area
    ax.set_position([0.25, 0.1, 0.7, 0.8])
    ax.set_box_aspect((1, 1, 1))

    # Adjust limits
    max_range = np.array([
        UAV['limt']['x'][1] - UAV['limt']['x'][0],
        UAV['limt']['y'][1] - UAV['limt']['y'][0],
        UAV['limt']['z'][1] - UAV['limt']['z'][0]
    ]).max() / 2.0

    mid_x = np.mean(UAV['limt']['x'])
    mid_y = np.mean(UAV['limt']['y'])
    mid_z = np.mean(UAV['limt']['z'])
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Customize axis appearance
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = False
        pane.set_edgecolor(COLORS['main']['primary4'])

    # Grid settings
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis._axinfo['grid']['color'] = COLORS['main']['primary4']
        axis._axinfo['grid']['linestyle'] = '--'
        axis.line.set_color('none')
        
    ax.grid(True)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_zticklabels([])
    ax.tick_params(colors=COLORS['main']['primary3'], labelsize=0, length=0)

def setup_3d_plot(figsize=(19.2, 10.8)):
    """Create and return a 3D plot with the correct configuration."""
    import matplotlib.pyplot as plt
    
    fig = plt.figure(figsize=figsize, dpi=DPI)
    ax = fig.add_subplot(111, projection='3d')
    return fig, ax

def add_colorbar(fig, ax):
    """Add a custom colorbar to the plot."""
    import matplotlib.pyplot as plt
    
    custom_cmap = plt.cm.colors.LinearSegmentedColormap.from_list("", [
        COLORS['secondary']['red'],
        COLORS['secondary']['yellow'],
        COLORS['secondary']['green']
    ])
    sm = plt.cm.ScalarMappable(cmap=custom_cmap, norm=plt.Normalize(vmin=0, vmax=1))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, label='Solution Quality', pad=0.1, aspect=30)
    cbar.set_ticks([0, 0.5, 1])
    cbar.set_ticklabels(['Worst', 'Medium', 'Best'])
    
    # Style the colorbar
    cbar.ax.yaxis.set_tick_params(color=COLORS['main']['primary3'])
    cbar.outline.set_edgecolor(COLORS['main']['primary3'])
    plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color=COLORS['main']['primary3'])
    cbar.set_label('Solution Quality', color=COLORS['main']['primary3'], 
                   fontweight='bold', fontsize=14)