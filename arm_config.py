"""Shared geometry + kinematics for the EOG-controlled arm simulation.

Used by both generate_world.py (builds the Gazebo SDF) and eog_arm_sim.py
(runs the EOG pipeline and drives the arm), so the two always agree.

Joint-angle convention (planar chain, rotation about the arm's local +Y axis):
    q = 0 means the link points straight up (+Z); positive q tilts it forward (+X).
"""
import math

# ---- arm geometry (metres) -------------------------------------------------
H0 = 0.15          # height of the shoulder-lift axis above the table
L1 = 0.25          # upper arm length (shoulder -> elbow)
L2 = 0.25          # forearm length   (elbow    -> wrist)
LT = 0.16          # wrist joint -> grasp point between the finger pads
FINGER_OPEN_HALF_GAP = 0.05     # half distance between finger pads when open
FINGER_STROKE = 0.04            # how far each finger can slide inward

# ---- task layout -------------------------------------------------------------
CUBE_SIZE = 0.04
PICK_ANGLE = 0.80     # rad  (positive = "Left" of the home position)
PLACE_ANGLE = -0.80   # rad  (negative = "Right")
TASK_RADIUS = 0.30    # distance of cube / goal pad from the base axis
GRASP_Z = 0.045       # height of the grasp point when the arm is lowered

# Home and "lowered" gripper positions in the arm's vertical plane (r, z).
HOME_RZ = (0.20, 0.28)
LOW_RZ = (TASK_RADIUS, GRASP_Z)

# ---- joint limits (rad / m) --------------------------------------------------
LIMITS = {
    "shoulder_pan":  (-2.6, 2.6),
    "shoulder_lift": (-0.6, 1.9),
    "elbow":         (0.0, 2.6),
    "wrist":         (-1.0, 3.0),
    "finger_left":   (0.0, FINGER_STROKE),
    "finger_right":  (0.0, FINGER_STROKE),
}
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow", "wrist",
               "finger_left", "finger_right"]


def ik_down(r, z):
    """Joint angles that put the grasp point at (r, z) with the gripper pointing straight down.

    Returns (shoulder_lift, elbow, wrist) in radians (elbow-up solution).
    """
    wx = r
    wz = z + LT - H0                      # wrist joint relative to the shoulder axis
    d2 = wx * wx + wz * wz
    c2 = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    q2 = math.acos(c2)
    phi = math.atan2(wx, wz)              # angle of the target from vertical
    q1 = phi - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    q3 = math.pi - q1 - q2                # keeps the gripper vertical
    return q1, q2, q3


def fk_down(q1, q2, q3):
    """Grasp point (r, z) for the given planar joint angles (sanity check)."""
    ex, ez = L1 * math.sin(q1), L1 * math.cos(q1)
    wx, wz = ex + L2 * math.sin(q1 + q2), ez + L2 * math.cos(q1 + q2)
    a = q1 + q2 + q3
    return wx + LT * math.sin(a), H0 + wz + LT * math.cos(a)


def path_rz(s):
    """Reach/lower parameter s in [0,1] -> (r, z) on the straight line HOME -> LOW."""
    s = max(0.0, min(1.0, s))
    return (HOME_RZ[0] + s * (LOW_RZ[0] - HOME_RZ[0]),
            HOME_RZ[1] + s * (LOW_RZ[1] - HOME_RZ[1]))


if __name__ == "__main__":
    for s in (0.0, 0.5, 1.0):
        r, z = path_rz(s)
        q = ik_down(r, z)
        print(f"s={s:.1f}  target=({r:.3f},{z:.3f})  q={[round(x,3) for x in q]}  "
              f"fk={[round(x,3) for x in fk_down(*q)]}")
        for name, v in zip(("shoulder_lift", "elbow", "wrist"), q):
            lo, hi = LIMITS[name]
            assert lo <= v <= hi, (name, v)
    print("IK/FK check OK")
