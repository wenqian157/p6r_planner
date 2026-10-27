"""AgileX UMR numbers used by the P6R models (single source of truth).

Sources (retrieved 2026-10-25):
  Generation Robots datasheet  https://static.generation-robots.com/media/datasheet-umr-agilex-robotics.pdf
  Generation Robots product page https://www.generationrobots.com/en/404270-umr-mobile-robot.html
  MyBotShop technical data      https://www.mybotshop.de/AgileX-UMR_1
No public drawing gives the arm mounting position; the mount values below are placeholders
to be replaced by lab measurements.
"""

# chassis (datasheet)
UMR_LENGTH = 0.830        # m, along the driving (x) direction
UMR_WIDTH = 0.540         # m
UMR_HEIGHT = 0.410        # m, top deck above the ground
UMR_GROUND_CLEARANCE = 0.090   # m
UMR_MASS = 90.0           # kg
UMR_PAYLOAD = 80.0        # kg
UMR_MAX_SPEED = 1.5       # m/s, bounds dx/ds in the (s, x) plane

# arm mount (placeholder: UR base_link centred on the deck on a 10 mm adapter plate)
ADAPTER_PLATE = 0.010                     # m
MOUNT_XYZ = (0.0, 0.0, UMR_HEIGHT + ADAPTER_PLATE)   # UR base_link origin in the UMR base frame
MOUNT_RPY = (0.0, 0.0, 0.0)               # UR base_link x aligned with UMR forward x

# prismatic base joint: UMR base frame slides along world x
X_TRAVEL = (-1.5, 1.5)    # m, covers the x = -1, 0, +1 m map positions of the proposal

# joint vector ordering for every P6R model:  q = (x, q1, q2, q3, q4, q5, q6)
JOINT_NAMES = ["base_x", "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
