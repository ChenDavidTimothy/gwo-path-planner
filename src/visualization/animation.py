import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Line3D, Poly3DCollection
import time
from tqdm import tqdm

from .config import (
    COLORS, DPI, FPS, DURATION, ROTATION_ANGLE,
    setup_3d_plot, setup_plot_style, add_colorbar
)
from .utils import create_cylinder, precompute_animation_data

class PathAnimator:
    def __init__(self, UAV, obj_fun):
        self.UAV = UAV
        self.obj_fun = obj_fun
        self.fig, self.ax = setup_3d_plot()
        setup_plot_style(self.ax, self.fig, UAV)
        self._setup_scene()

    def _setup_scene(self):
        """Setup the basic scene elements."""
        # Add start and end points
        self.start = self.ax.scatter(*self.UAV['S'], c=COLORS['secondary']['green'],
                                   marker='o', s=150, label='Start',
                                   edgecolors=COLORS['main']['primary3'], linewidth=2)
        self.end = self.ax.scatter(*self.UAV['G'], c=COLORS['secondary']['red'],
                                 marker='s', s=150, label='End',
                                 edgecolors=COLORS['main']['primary3'], linewidth=2)
        
        # Add no-fly zones
        self.no_fly_zones = []
        for zone in self.UAV['NoFlyZones']:
            x, y, height, radius = zone
            X, Y, Z, X_top, Y_top, Z_top = create_cylinder(x, y, height, radius)
            
            cylinder = self.ax.plot_surface(X, Y, Z, color=COLORS['main']['primary1'],
                                          alpha=0.2, linewidth=0)
            self.no_fly_zones.append(cylinder)
            
            top_circle = self.ax.plot(X_top, Y_top, Z_top, 
                                    color=COLORS['main']['primary1'], alpha=0.1)
            self.ax.add_collection3d(Poly3DCollection(
                [list(zip(X_top, Y_top, Z_top))],
                color=COLORS['main']['primary1'], alpha=0.2
            ))
            self.no_fly_zones.append(top_circle)

        # Add legend
        self.ax.legend(loc='upper left', bbox_to_anchor=(-0.3, 1),
                      facecolor=COLORS['main']['background'],
                      edgecolor=COLORS['main']['primary3'],
                      labelcolor=COLORS['main']['primary3'], fontsize=12)

        # Add colorbar
        add_colorbar(self.fig, self.ax)

    def create_animation(self, all_paths, fps=FPS, duration=DURATION, 
                        rotation_angle=ROTATION_ANGLE):
        """Create the animation."""
        precomputed_data = precompute_animation_data(
            all_paths, self.UAV, self.obj_fun, fps, duration
        )
        
        num_wolves = len(precomputed_data[0]['frame_data'])
        self.paths = [Line3D([], [], [], alpha=1, linewidth=2) 
                     for _ in range(num_wolves)]
        for path in self.paths:
            self.ax.add_line(path)
        
        self.nav_points = [self.ax.plot([], [], [], 'o', markersize=6, alpha=1)[0] 
                          for _ in range(num_wolves)]

        # Add text elements
        self.fitness_text = self.fig.text(
            0.1, 0.65, '', fontsize=14,
            color=COLORS['main']['primary1'], fontweight='bold'
        )
        self.iteration_text = self.fig.text(
            0.1, 0.60, '', fontsize=14,
            color=COLORS['main']['primary1'], fontweight='bold'
        )

        total_frames = len(precomputed_data)

        def animate(frame):
            frame_data = precomputed_data[frame]['frame_data']
            
            for i, wolf_data in enumerate(frame_data):
                self.paths[i].set_data_3d(*wolf_data['path'])
                self.paths[i].set_color(wolf_data['color'])
                
                self.nav_points[i].set_data(wolf_data['nav_points'][0],
                                          wolf_data['nav_points'][1])
                self.nav_points[i].set_3d_properties(wolf_data['nav_points'][2])
                self.nav_points[i].set_color(COLORS['secondary']['blue'])
            
            self.fitness_text.set_text(
                f'Best Fitness: {precomputed_data[frame]["best_fitness"]:.2f}'
            )
            self.iteration_text.set_text(
                f'Iteration: {precomputed_data[frame]["iteration"] + 1}/{len(all_paths)}'
            )
            
            # Update view angle for rotation
            azimuth = (frame / total_frames) * rotation_angle
            self.ax.view_init(elev=20, azim=azimuth)
            
            return self.paths + self.nav_points + [self.fitness_text, self.iteration_text]

        anim = FuncAnimation(
            self.fig, animate, frames=total_frames,
            interval=1000/fps, blit=False
        )

        return anim, total_frames

def save_animation(anim, total_frames, filename='path_animation.mp4', fps=FPS):
    """Save the animation to a file."""
    writer = animation.FFMpegWriter(
        fps=fps,
        metadata=dict(artist='GWO Path Planner'),
        bitrate=10_000,
        codec='libx264',
        extra_args=['-preset', 'slow', '-crf', '15', '-pix_fmt', 'yuv420p']
    )
    
    print("Saving animation...")
    with tqdm(total=total_frames, desc="Rendering frames") as pbar:
        anim.save(filename, writer=writer,
                 progress_callback=lambda i, n: pbar.update(1))
    
    plt.close(anim._fig)