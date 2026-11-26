"""Day 33: create targets and a simple program in RoboDK from Python.

Open RoboDK first and load a robot (File > Open online library). Then run this file.
Poses are in mm and degrees, the RoboDK way.
NOT TESTED against a live RoboDK by the author: if a call fails, the message is printed.
"""
from robodk import robomath as rm
from robodk.robolink import ITEM_TYPE_ROBOT, Robolink

# Offsets from the robot's current pose, in mm: (dx, dy, dz)
OFFSETS = [
    ("Home", (0, 0, 0)),
    ("Above_part", (0, 150, 0)),
    ("Weld_point", (0, 150, -100)),
    ("Retreat", (0, 150, 50)),
]


def build_poses(start_pose, offsets):
    """Return a list of (name, pose): each is the start pose moved by an offset in mm."""
    return [(name, start_pose * rm.transl(dx, dy, dz)) for name, (dx, dy, dz) in offsets]


def main():
    rdk = Robolink()
    robots = rdk.ItemList(ITEM_TYPE_ROBOT)
    if not robots:
        raise SystemExit("No robot found. Load one in RoboDK first.")
    robot = robots[0]
    print("Using robot:", robot.Name())

    start = robot.Pose()
    targets = []
    for name, pose in build_poses(start, OFFSETS):
        joints = robot.SolveIK(pose)
        if joints is None or len(joints.list()) == 0:
            print(f"  {name}: NOT reachable, skipped")
            continue
        target = rdk.AddTarget(name, 0, robot)
        target.setPose(pose)
        targets.append(target)
        print(f"  {name}: created")

    if len(targets) < 2:
        raise SystemExit("Need at least two reachable targets to make a program.")

    prog = rdk.AddProgram("Day33_Program", robot)
    prog.MoveJ(targets[0])            # joint move to the first target
    for target in targets[1:]:
        prog.MoveL(target)            # straight-line moves between the others
    print(f"Program 'Day33_Program' created with {len(targets)} targets.")
    print("Double-click the program in the station tree to run it.")


if __name__ == "__main__":
    try:
        main()
    except SystemExit as stop:
        print(stop)
    except Exception as err:
        print("RoboDK step failed:", type(err).__name__, "-", err)