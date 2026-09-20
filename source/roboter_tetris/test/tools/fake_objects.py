#!/usr/bin/env python3
"""Synthetic `objects` (S1) generator -- drives the data path without hardware.

Publishes the same signal `base_cam` produces, so `vectoring`,
`priority_handler` and the `object_follower` can be exercised **without
cameras, without the robot and without the setup**. Smoothing, plausibility,
selection, reachability, target lock and the whole state machine become
testable at the desk.

Not an AICA component on purpose: it is a plain ROS 2 publisher, so it needs no
rebuild of the package and no reload of the application.

    # print frames to stdout -- needs no ROS at all
    python3 fake_objects.py --dry-run --duration 20

    # publish into a running system (topic must match the AICA wiring)
    python3 fake_objects.py --topic /base_kamera/objects

Finding the topic at the setup
------------------------------
AICA names the topic after the component instance, so the default above only
fits if the block is called `base_kamera`. List them inside the container --
and note the two traps: it must run as user `ros2` (FastDDS hands the shared
memory segments to their owner, so as root the topics are visible but no data
arrives), and an empty ROS_DOMAIN_ID crashes the CLI:

    docker exec -u ros2 -e ROS_DOMAIN_ID=0 <container> \
        bash -lc 'ros2 topic list | grep objects'

Run `base_cam` and this tool at the same time only if you want to see them
fight -- normally you stop the camera component first.

The scene model (`FakeBelt`) is free of ROS imports so it can be unit-tested.

Defaults and where they come from
---------------------------------
Everything measured at the setup is used; everything still open (B1) is a
documented guess and a command-line option.

* **Belt direction is -y.** A hand-move "down the belt" on 15.09.2026 came out
  as -275.5 mm in y against +32.7 mm in x, so the belt runs along the y axis.
* **Lateral position x = 0.814 m.** A block on the belt measured on 14.09.2026
  at x = 814, y = -874 mm; that is roughly the middle of the belt.
* **Spawn/despawn in y** follow the tracker window of the predecessor group:
  it deletes objects outside y = +375 ... -1080 mm.
* **z = belt surface + height/2.** The belt surface is at z = 53.6 mm (B17).
  Note the contract calls S1 field 4 the block's *top edge*, but the
  measurement disagrees: base_cam reported 103.1 mm for a 100 mm block, and
  0.0536 + 0.050 = 0.1036 m matches that to within 0.5 mm. This generator
  reproduces what the camera actually emits, not what the table claims.
* **Belt speed 0.1 m/s is a GUESS** -- B1 is open. The tracker discards
  anything below 30 mm/s and reports 0, so stay above that.
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence

import os
import sys

# test/tools -> the package root that holds the roboter_tetris/ package.
sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..")))
from roboter_tetris.contracts import (  # noqa: E402
    COLOR_BLUE, COLOR_RED, COLOR_WHITE, ObjectEntry, pack_objects,
)

BELT_SURFACE_Z_M = 0.0536      # B17, measured 15.09.2026
BELT_CENTER_X_M = 0.814        # measured block position, 14.09.2026
Y_SPAWN_M = 0.300              # inside the tracker window (+375 mm)
Y_DESPAWN_M = -1.080           # tracker deletes beyond this
DEFAULT_V_BELT_MPS = -0.100    # GUESS, B1 open; negative = along -y


@dataclass
class FakeBlock:
    """One block on the imaginary belt. Lengths in metres."""

    id: int
    color: int
    length: float
    width: float
    height: float
    spawn_t: float = 0.0
    x_m: float = BELT_CENTER_X_M
    orientation: float = 0.0
    #: Stop the block here for good -- used to produce plausibility status 1
    #: ("stalled / stuck") downstream in `vectoring`.
    stall_at_y_m: Optional[float] = None


def default_blocks() -> List[FakeBlock]:
    """Three blocks, staggered so they appear one after another.

    The sizes are chosen to exercise the graspability check in
    `priority_handler`: the 100 mm block is the reference, the flat one sits
    just above the ~24 mm lower bound from B15, and the wide one approaches the
    127 mm gripper opening from B16.
    """
    return [
        FakeBlock(id=1, color=COLOR_RED, length=0.050, width=0.050,
                  height=0.100, spawn_t=0.0, orientation=0.0),
        FakeBlock(id=2, color=COLOR_BLUE, length=0.076, width=0.050,
                  height=0.050, spawn_t=4.0, orientation=0.6),
        FakeBlock(id=3, color=COLOR_WHITE, length=0.025, width=0.025,
                  height=0.025, spawn_t=8.0, orientation=1.2),
    ]


class FakeBelt:
    """Moves blocks along the belt and renders them as an S1 signal.

    Pure computation: no ROS, no clock of its own. The caller supplies the
    time, which makes the whole model reproducible and unit-testable.
    """

    def __init__(self, blocks: Sequence[FakeBlock],
                 v_belt_mps: float = DEFAULT_V_BELT_MPS,
                 y_spawn_m: float = Y_SPAWN_M,
                 y_despawn_m: float = Y_DESPAWN_M,
                 belt_surface_z_m: float = BELT_SURFACE_Z_M) -> None:
        self.blocks = list(blocks)
        self.v_belt_mps = v_belt_mps
        self.y_spawn_m = y_spawn_m
        self.y_despawn_m = y_despawn_m
        self.belt_surface_z_m = belt_surface_z_m

    def y_of(self, block: FakeBlock, t: float) -> Optional[float]:
        """Longitudinal position at time ``t``, or ``None`` if not on the belt."""
        if t < block.spawn_t:
            return None
        y = self.y_spawn_m + self.v_belt_mps * (t - block.spawn_t)
        if block.stall_at_y_m is not None:
            # Past the stall point the block sits still -- what a jammed or
            # toppled block looks like to the tracker.
            if self.v_belt_mps < 0.0:
                y = max(y, block.stall_at_y_m)
            else:
                y = min(y, block.stall_at_y_m)
        if self.v_belt_mps < 0.0 and y < self.y_despawn_m:
            return None
        if self.v_belt_mps > 0.0 and y > self.y_despawn_m:
            return None
        return y

    def objects_at(self, t: float) -> List[ObjectEntry]:
        out = []
        for block in self.blocks:
            y = self.y_of(block, t)
            if y is None:
                continue
            out.append(ObjectEntry(
                id=float(block.id), color=float(block.color),
                x=block.x_m, y=y,
                # See the module docstring: this mirrors what base_cam really
                # reports, which is mid-height, not the top edge.
                z=self.belt_surface_z_m + block.height / 2.0,
                orientation=block.orientation,
                length=block.length, width=block.width, height=block.height,
            ))
        return out

    def signal_at(self, t: float) -> List[float]:
        """The full S1 array for time ``t``."""
        return pack_objects(t, self.v_belt_mps, self.objects_at(t))


def build_parser():
    import argparse
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--topic", default="/base_kamera/objects",
                   help="ROS topic to publish on; must match the AICA wiring "
                        "of the base_cam 'objects' output (default: %(default)s)")
    p.add_argument("--rate", type=float, default=30.0,
                   help="publish rate in Hz (default: %(default)s, roughly the "
                        "29.7 Hz the real cameras deliver)")
    p.add_argument("--velocity", type=float, default=DEFAULT_V_BELT_MPS,
                   help="belt velocity in m/s along y; negative runs along -y "
                        "(default: %(default)s -- a GUESS, B1 is open)")
    p.add_argument("--stall-id", type=int, default=None,
                   help="stop this block permanently, to check that it ends up "
                        "with plausibility status 1 in vectoring")
    p.add_argument("--stall-at-y", type=float, default=-0.700,
                   help="y in m where --stall-id stops (default: %(default)s, "
                        "inside the tracker measurement region)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="stop after this many seconds (0 = run forever)")
    p.add_argument("--dry-run", action="store_true",
                   help="print frames to stdout instead of publishing; needs no ROS")
    return p


def build_belt(args) -> FakeBelt:
    blocks = default_blocks()
    if args.stall_id is not None:
        for b in blocks:
            if b.id == args.stall_id:
                b.stall_at_y_m = args.stall_at_y
                break
        else:
            raise SystemExit(f"--stall-id {args.stall_id}: kein solcher Block "
                             f"(vorhanden: {[b.id for b in blocks]})")
    return FakeBelt(blocks, v_belt_mps=args.velocity)


def run_dry(belt: FakeBelt, args) -> None:
    """Print a few frames so the scene can be checked without any ROS."""
    duration = args.duration or 20.0
    step = 1.0 / args.rate
    print(f"# v_belt={belt.v_belt_mps} m/s  rate={args.rate} Hz  "
          f"duration={duration} s")
    print("# t[s]  n  ids @ y[m]")
    t = 0.0
    while t <= duration:
        objs = belt.objects_at(t)
        detail = "  ".join(f"{int(o.id)}@{o.y:+.3f}" for o in objs)
        print(f"{t:6.2f}  {len(objs)}  {detail}")
        t += max(step, 0.5)      # coarse steps: this is for eyeballing


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    belt = build_belt(args)
    if args.dry_run:
        run_dry(belt, args)
        return 0

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray

    class FakeObjectsNode(Node):
        def __init__(self):
            super().__init__("fake_objects")
            self._pub = self.create_publisher(Float64MultiArray, args.topic, 10)
            self._t0 = self.get_clock().now()
            self.create_timer(1.0 / args.rate, self._tick)
            self.get_logger().info(
                f"fake_objects -> {args.topic} @ {args.rate} Hz, "
                f"v_belt={belt.v_belt_mps} m/s")

        def _tick(self):
            t = (self.get_clock().now() - self._t0).nanoseconds / 1e9
            if args.duration and t > args.duration:
                raise SystemExit
            msg = Float64MultiArray()
            # S1 carries the frame timestamp; here that is simply run time.
            msg.data = belt.signal_at(t)
            self._pub.publish(msg)

    rclpy.init()
    node = FakeObjectsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
