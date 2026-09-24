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
Most defaults are from before the measurements of 22./23.09.2026 and still
work as test values; where the setup is known now, it is noted.

**All positions are in the robot frame** (`world` of the robot, Nachtrag 8 /
F1), because the follower moves the robot there. Since 23.09.2026 the base
camera reports in the same frame (B23 done, Nachtrag 13 / L6).

* **Belt along -y.** Measured since: the belt runs from y = +1.08 to -0.375 m
  (Nachtrag 13 / L7), so -y is right.
* **Lateral position x = -0.816 m**, the middle of the belt surface the robot
  touched at x = -0.70 ... -0.93 (M9).
* **Spawn/despawn at y = +0.60 / -0.80** -- a test stretch. The real belt ends
  at y = -0.375; the grasp zone lies outside the base camera image, about
  y = 0.30 ... -0.30 (Nachtrag 13 / L4).
* **z = belt surface + height/2**, the belt surface at z = 53.6 mm (B17) -- S1
  field 4 is mid-height (contract, since 23.09.2026 also in `base_cam`).
* **Belt speed 0.1 m/s** -- the stopwatch gives about 0.13 m/s (L7). The
  tracker discards anything below 30 mm/s and reports 0, so stay above that.
* **Rate 30 Hz, no latency** -- the real base camera delivers ~7 measurements/s,
  each ~0.14 s old on arrival (Nachtrag 13 / L2). A realistic run needs a lower
  ``--rate``; an option for the latency is still open.

Two things a real belt does that matter for testing `vectoring`
----------------------------------------------------------------
* **Measurement noise** (`noise_sigma_m`). Without it every velocity estimate is
  trivially exact and a test proves nothing. base_cam scattered by 0.2...0.6 mm
  on a resting block (M5); a moving one is not measured yet.
* **Toppling on placement** (`Topple`, `--topple-id`). The only disturbance that
  actually happens: blocks are placed freely and run with the belt, but one may
  fall over while settling. Its centre jumps and its shape changes. There is no
  "stalled" block -- jammed or knocked blocks do not occur (Nachtrag 6 / Z3).
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence

import os
import random
import sys

# test/tools -> the package root that holds the roboter_tetris/ package.
sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..")))
from roboter_tetris.contracts import (  # noqa: E402
    COLOR_BLUE, COLOR_RED, COLOR_WHITE, ObjectEntry, pack_objects,
)

BELT_SURFACE_Z_M = 0.0536      # B17, measured 15.09.2026
# Robot frame (Nachtrag 8 / F1).
BELT_CENTER_X_M = -0.816       # middle of the touched belt surface, M9
Y_SPAWN_M = 0.600              # GUESS: upstream end of the simulated stretch
Y_DESPAWN_M = -0.800           # GUESS: downstream end
DEFAULT_V_BELT_MPS = -0.100    # GUESS, B1 open; negative = along -y


@dataclass
class Topple:
    """A block falling over while it settles after placement.

    From ``after_s`` (seconds after spawn) on, its centre is shifted and its
    shape is the lying one. Build it with :func:`topple_forward` rather than by
    hand -- that gets the geometry right.
    """

    after_s: float
    shift_along_m: float          # centre jump along the belt's running direction
    length: float                 # shape once lying
    width: float
    height: float


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
    topple: Optional[Topple] = None


def topple_forward(block: FakeBlock, after_s: float = 0.3) -> Topple:
    """A standing block tipping over its front edge, in the running direction.

    It pivots about the bottom edge; lying, it is ``height`` long. Its centre was
    ``length/2`` from that edge and ends up ``height/2`` from it, so it jumps by
    ``(height - length) / 2`` -- 25 mm for the 50 x 50 x 100 reference block.
    """
    if block.height <= block.length:
        raise ValueError(
            f"Klotz {block.id} steht nicht hochkant "
            f"({block.length:.3f} x {block.width:.3f} x {block.height:.3f} m) "
            "und kann nicht nach vorn umkippen")
    return Topple(after_s=after_s,
                  shift_along_m=(block.height - block.length) / 2.0,
                  length=block.height, width=block.width, height=block.length)


def default_blocks() -> List[FakeBlock]:
    """Three blocks, staggered so they appear one after another.

    The sizes are chosen to exercise the graspability check in
    `priority_handler`: the 100 mm block is the reference and the only one that
    can topple, the wide one approaches the 127 mm gripper opening (B16), and
    the 25 mm cube is **deliberately below** the ~30 mm graspability limit
    (B15 with 5 mm air, Nachtrag 6 / Z7) -- a negative case: it must never be
    selected.
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
                 belt_surface_z_m: float = BELT_SURFACE_Z_M,
                 noise_sigma_m: float = 0.0,
                 seed: Optional[int] = None) -> None:
        self.blocks = list(blocks)
        self.v_belt_mps = v_belt_mps
        self.y_spawn_m = y_spawn_m
        self.y_despawn_m = y_despawn_m
        self.belt_surface_z_m = belt_surface_z_m
        #: Gaussian noise on x, y and z of every reported object. Drawn from a
        #: private generator: the same seed and the same call sequence give the
        #: same scene. Two calls for the same ``t`` give different noise.
        self.noise_sigma_m = noise_sigma_m
        self._rng = random.Random(seed)

    def y_of(self, block: FakeBlock, t: float) -> Optional[float]:
        """Longitudinal position at time ``t``, or ``None`` if not on the belt."""
        if t < block.spawn_t:
            return None
        y = self.y_spawn_m + self.v_belt_mps * (t - block.spawn_t)
        if self._toppled(block, t):
            # The centre jumped forward, i.e. in the running direction.
            y += block.topple.shift_along_m * (1.0 if self.v_belt_mps > 0.0 else -1.0)
        if self.v_belt_mps < 0.0 and y < self.y_despawn_m:
            return None
        if self.v_belt_mps > 0.0 and y > self.y_despawn_m:
            return None
        return y

    @staticmethod
    def _toppled(block: FakeBlock, t: float) -> bool:
        return (block.topple is not None
                and t - block.spawn_t >= block.topple.after_s)

    def _noise(self) -> float:
        if self.noise_sigma_m <= 0.0:
            return 0.0
        return self._rng.gauss(0.0, self.noise_sigma_m)

    def objects_at(self, t: float) -> List[ObjectEntry]:
        out = []
        for block in self.blocks:
            y = self.y_of(block, t)
            if y is None:
                continue
            if self._toppled(block, t):
                length, width, height = (block.topple.length, block.topple.width,
                                         block.topple.height)
            else:
                length, width, height = block.length, block.width, block.height
            out.append(ObjectEntry(
                id=float(block.id), color=float(block.color),
                x=block.x_m + self._noise(), y=y + self._noise(),
                # See the module docstring: this mirrors what base_cam really
                # reports, which is mid-height, not the top edge.
                z=self.belt_surface_z_m + height / 2.0 + self._noise(),
                orientation=block.orientation,
                length=length, width=width, height=height,
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
    p.add_argument("--topple-id", type=int, default=None,
                   help="let this block fall over shortly after placement, to "
                        "check that vectoring keeps it settling (status 3) until "
                        "it runs evenly again")
    p.add_argument("--topple-after", type=float, default=0.3,
                   help="seconds after placement when --topple-id lands on its "
                        "side (default: %(default)s)")
    p.add_argument("--noise", type=float, default=0.0005,
                   help="measurement noise in m on x, y, z (default: %(default)s, "
                        "the 0.5 mm base_cam showed on a resting block)")
    p.add_argument("--seed", type=int, default=None,
                   help="seed for the noise, for a reproducible scene")
    p.add_argument("--duration", type=float, default=0.0,
                   help="stop after this many seconds (0 = run forever)")
    p.add_argument("--dry-run", action="store_true",
                   help="print frames to stdout instead of publishing; needs no ROS")
    return p


def build_belt(args) -> FakeBelt:
    blocks = default_blocks()
    if args.topple_id is not None:
        for b in blocks:
            if b.id == args.topple_id:
                try:
                    b.topple = topple_forward(b, after_s=args.topple_after)
                except ValueError as exc:
                    raise SystemExit(f"--topple-id {args.topple_id}: {exc}")
                break
        else:
            raise SystemExit(f"--topple-id {args.topple_id}: kein solcher Block "
                             f"(vorhanden: {[b.id for b in blocks]})")
    return FakeBelt(blocks, v_belt_mps=args.velocity,
                    noise_sigma_m=args.noise, seed=args.seed)


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
            # The scene runs in seconds since start, but S1 carries the image
            # time in ROS time like base_cam: the follower ages every target
            # against its own clock and rejected run-time stamps as ~56 years
            # old (24.09.2026, Nachtrag 13 / L17).
            data = belt.signal_at(t)
            data[0] = self.get_clock().now().nanoseconds / 1e9
            msg.data = data
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
