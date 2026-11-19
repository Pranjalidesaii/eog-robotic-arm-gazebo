"""Generates eog_arm_world.sdf (Gazebo Harmonic) for the EOG-controlled 4-DOF arm.

Run:  python3 generate_world.py
"""
import math
import os
import re
from arm_config import *

WORLD = "eog_world"
MODEL = "eog_arm"
ARM_GRAVITY = False
HERE = os.path.dirname(os.path.abspath(__file__))


def box_inertia(m, x, y, z):
    return (m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y))


def cyl_inertia(m, r, h):
    return (m / 12 * (3 * r * r + h * h), m / 12 * (3 * r * r + h * h), m / 2 * r * r)


def inertial(m, ixx, iyy, izz, cz=0.0):
    return f"""<inertial><pose>0 0 {cz} 0 0 0</pose><mass>{m}</mass>
        <inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"""


def mat(r, g, b, a=1.0):
    return (f"<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>"
            f"<specular>0.3 0.3 0.3 1</specular></material>")


def box_geom(x, y, z):
    return f"<geometry><box><size>{x} {y} {z}</size></box></geometry>"


def cyl_geom(r, h):
    return f"<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>"


def part(name, geom, color, pose="0 0 0 0 0 0", collide=True, friction=None, mask=None):
    """mask = collide_bitmask. Arm links get unique bits so they never collide with each
    other (only with the table); fingers + cube share bit 0x01 so grasping works."""
    surf = ""
    if friction or mask is not None:
        fr = (f"<friction><ode><mu>{friction}</mu><mu2>{friction}</mu2></ode></friction>"
              if friction else "")
        mk = f"<contact><collide_bitmask>{mask:#06x}</collide_bitmask></contact>" if mask is not None else ""
        surf = f"<surface>{fr}{mk}</surface>"
    col = f"<collision name='{name}_col'><pose>{pose}</pose>{geom}{surf}</collision>" if collide else ""
    return f"{col}<visual name='{name}_vis'><pose>{pose}</pose>{geom}{mat(*color)}</visual>"


def joint(name, parent, child, kind, axis, lo, hi, pose="0 0 0 0 0 0", damping=0.05, effort=60):
    pose = "0 0 0 0 0 0"   # joint frame == child link frame (children are placed at their joint)
    ax = ""
    if axis:
        ax = (f"<axis><xyz>{axis}</xyz><limit><lower>{lo}</lower><upper>{hi}</upper>"
              f"<effort>{effort}</effort><velocity>3</velocity></limit>"
              f"<dynamics><damping>{damping}</damping></dynamics></axis>")
    return f"<joint name='{name}' type='{kind}'><parent>{parent}</parent><child>{child}</child><pose>{pose}</pose>{ax}</joint>"


def controller(jname, p, i, d, cmax):
    return f"""<plugin filename="gz-sim-joint-position-controller-system"
        name="gz::sim::systems::JointPositionController">
      <joint_name>{jname}</joint_name>
      <topic>/{MODEL}/{jname}/cmd_pos</topic>
      <p_gain>{p}</p_gain><i_gain>{i}</i_gain><d_gain>{d}</d_gain>
      <i_max>50</i_max><i_min>-50</i_min>
      <cmd_max>{cmax}</cmd_max><cmd_min>-{cmax}</cmd_min>
    </plugin>"""


def build_arm():
    links = []
    joints = []

    # ---- fixed base (pedestal) -------------------------------------------
    links.append(f"""<link name='base_link'>{inertial(2.0, *cyl_inertia(2.0, .07, .05), cz=.025)}
      {part('base', cyl_geom(.07, .05), (.18, .18, .2), '0 0 .025 0 0 0', mask=0x08)}</link>""")
    joints.append(joint("world_fixed", "world", "base_link", "fixed", None, 0, 0))

    # ---- turret: shoulder pan (about Z) ------------------------------------
    tur_h = H0 - 0.05
    links.append(f"""<link name='turret'><pose>0 0 .05 0 0 0</pose>
      {inertial(0.5, *cyl_inertia(.5, .05, tur_h), cz=tur_h / 2)}
      {part('turret', cyl_geom(.05, tur_h), (.1, .35, .75), f'0 0 {tur_h / 2} 0 0 0', mask=0x10)}
      {part('shoulder_hub', cyl_geom(.035, .075), (.8, .8, .85), f'0 0 {tur_h} 1.5708 0 0', collide=False)}
    </link>""")
    lo, hi = LIMITS["shoulder_pan"]
    joints.append(joint("shoulder_pan", "base_link", "turret", "revolute", "0 0 1", lo, hi,
                        pose="0 0 .05 0 0 0", damping=0.3))

    # ---- upper arm (shoulder lift about Y) -----------------------------------
    m = 0.25
    links.append(f"""<link name='upper_arm'><pose>0 0 {H0} 0 0 0</pose>
      {inertial(m, *box_inertia(m, .04, .05, L1), cz=L1 / 2)}
      {part('upper_arm', box_geom(.04, .05, L1), (.95, .55, .1), f'0 0 {L1 / 2} 0 0 0', mask=0x20)}
      {part('elbow_hub', cyl_geom(.03, .06), (.8, .8, .85), f'0 0 {L1} 1.5708 0 0', collide=False)}
    </link>""")
    lo, hi = LIMITS["shoulder_lift"]
    joints.append(joint("shoulder_lift", "turret", "upper_arm", "revolute", "0 1 0", lo, hi,
                        pose=f"0 0 {H0} 0 0 0", damping=0.5))

    # ---- forearm (elbow about Y) -----------------------------------------------
    m = 0.2
    links.append(f"""<link name='forearm'><pose>0 0 {H0 + L1} 0 0 0</pose>
      {inertial(m, *box_inertia(m, .035, .04, L2), cz=L2 / 2)}
      {part('forearm', box_geom(.035, .04, L2), (.2, .65, .35), f'0 0 {L2 / 2} 0 0 0', mask=0x40)}
      {part('wrist_hub', cyl_geom(.025, .05), (.8, .8, .85), f'0 0 {L2} 1.5708 0 0', collide=False)}
    </link>""")
    lo, hi = LIMITS["elbow"]
    joints.append(joint("elbow", "upper_arm", "forearm", "revolute", "0 1 0", lo, hi,
                        pose=f"0 0 {H0 + L1} 0 0 0", damping=0.4))

    # ---- wrist + palm (wrist about Y); local +Z is the gripper direction ---------
    m = 0.1
    palm_len = LT - 0.035
    links.append(f"""<link name='wrist_link'><pose>0 0 {H0 + L1 + L2} 0 0 0</pose>
      {inertial(m, *box_inertia(m, .03, .1, palm_len), cz=palm_len / 2)}
      {part('palm', box_geom(.03, .12, .03), (.85, .85, .9), f'0 0 {palm_len + .01} 0 0 0', mask=0x80)}
      {part('wrist_shaft', box_geom(.025, .03, palm_len), (.5, .5, .55), f'0 0 {palm_len / 2} 0 0 0', mask=0x80)}
    </link>""")
    lo, hi = LIMITS["wrist"]
    joints.append(joint("wrist", "forearm", "wrist_link", "revolute", "0 1 0", lo, hi,
                        pose=f"0 0 {H0 + L1 + L2} 0 0 0", damping=0.15, effort=20))

    # ---- two fingers sliding along the wrist's Y axis ---------------------------------
    fw, flen = 0.012, 0.07
    fy = FINGER_OPEN_HALF_GAP + fw / 2
    for side, sgn in (("left", 1), ("right", -1)):
        links.append(f"""<link name='finger_{side}_link'>
        <pose>0 {sgn * fy} {H0 + L1 + L2 + LT} 0 0 0</pose>
        {inertial(0.03, *box_inertia(.03, .025, fw, flen))}
        {part('finger_' + side, box_geom(.025, fw, flen), (.9, .15, .15), friction=2.0, mask=0x01)}
      </link>""")
        lo, hi = LIMITS["finger_" + side]
        joints.append(joint("finger_" + side, "wrist_link", "finger_" + side + "_link", "prismatic",
                            f"0 {-sgn} 0", lo, hi, pose=f"0 {sgn * fy} 0 0 0 0",
                            damping=2.0, effort=10))
        # (joint pose is relative to the child link; child origin sits at the grasp point)

    plugins = [
        controller("shoulder_pan", 30, 0.0, 3, 20),
        controller("shoulder_lift", 60, 0.0, 5, 30),
        controller("elbow", 40, 0.0, 3, 25),
        controller("wrist", 4, 0.0, 0.2, 5),
        controller("finger_left", 200, 0, 5, 10),
        controller("finger_right", 200, 0, 5, 10),
        """<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher"/>""",
    ]
    body = "\n".join(links + joints + plugins)
    if not ARM_GRAVITY:   # idealised servos: the arm holds its pose without sag
        body = re.sub(r"<link name='(\w+)'>", r"<link name='\1'><gravity>false</gravity>", body)
    return f"<model name='{MODEL}'>\n" + body + "\n</model>"


def build_scene():
    cx, cy = TASK_RADIUS * math.cos(PICK_ANGLE), TASK_RADIUS * math.sin(PICK_ANGLE)
    gx, gy = TASK_RADIUS * math.cos(PLACE_ANGLE), TASK_RADIUS * math.sin(PLACE_ANGLE)
    cs = CUBE_SIZE
    cube = f"""<model name='target_cube'><pose>{cx:.4f} {cy:.4f} {cs / 2} 0 0 {PICK_ANGLE}</pose>
    <link name='cube'>{inertial(0.05, *box_inertia(.05, cs, cs, cs))}
      <collision name='c'>{box_geom(cs, cs, cs)}
        <surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction><contact><collide_bitmask>0x01</collide_bitmask></contact></surface></collision>
      <visual name='v'>{box_geom(cs, cs, cs)}{mat(.9, .1, .1)}</visual></link></model>"""
    pad = f"""<model name='goal_pad'><static>true</static>
    <pose>{gx:.4f} {gy:.4f} 0.0015 0 0 {PLACE_ANGLE}</pose>
    <link name='pad'><visual name='v'>{box_geom(.09, .09, .003)}{mat(.1, .75, .25)}</visual></link></model>"""
    home = f"""<model name='home_marker'><static>true</static><pose>0 0 0.001 0 0 0</pose>
    <link name='m'><visual name='v'><geometry><cylinder><radius>.095</radius><length>.002</length></cylinder></geometry>
    {mat(.25, .25, .3)}</visual></link></model>"""
    return cube + pad + home


def build_world():
    return f"""<?xml version="1.0" ?>
<!-- AUTO-GENERATED by generate_world.py -->
<sdf version="1.9">
  <world name="{WORLD}">
    <physics name="1ms" type="dart"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <gravity>0 0 -9.81</gravity>
    <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.78 0.85 0.93 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>1 -2 6 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse><specular>0.3 0.3 0.3 1</specular><direction>-0.4 0.3 -1</direction></light>

    <model name="table"><static>true</static>
      <link name="top">
        <collision name="c"><pose>0 0 -0.02 0 0 0</pose>{box_geom(1.4, 1.1, 0.04)}
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><pose>0 0 -0.02 0 0 0</pose>{box_geom(1.4, 1.1, 0.04)}{mat(.82, .78, .7)}</visual>
      </link></model>

    {build_arm()}
    {build_scene()}
  </world>
</sdf>
"""


if __name__ == "__main__":
    path = os.path.join(HERE, "eog_arm_world.sdf")
    with open(path, "w", encoding="utf-8") as f:
        f.write(build_world())
    print("wrote", path)
