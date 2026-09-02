"""
Tracked Unmanned Ground Vehicle (UGV) Physical Profile and Geometric Limits.
"""

from dataclasses import dataclass


@dataclass
class TrackedVehicleProfile:
    """
    Physical and dynamic envelope for continuous track mobility.
    """
    # Chassis clearance & vertical steps
    ground_clearance: float = 0.10       # 10 cm clearance under belly
    max_climb_step: float = 0.15         # 15 cm step climb capability with tracks
    max_drop_step: float = 0.12          # 12 cm maximum safe step-down
    soft_vegetation_max_height: float = 0.40 # 40 cm tall soft grass/brush that tracks can crush
    
    # Terrain gradients & spans
    max_slope_deg: float = 35.0          # 35 degree maximum traversable incline
    max_side_slope_deg: float = 25.0     # 25 degree rollover angle threshold
    max_trench_width: float = 0.25       # 25 cm ditch/trench span capability
    
    # Dimensions (meters)
    robot_length: float = 0.80           # Length along forward axis
    robot_width: float = 0.60            # Width track-to-track
    track_width: float = 0.15            # Individual track tread width
    inflation_radius: float = 0.35       # Safety inflation radius for costmap
