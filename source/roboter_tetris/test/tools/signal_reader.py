#!/usr/bin/env python3
"""Read-only probe for the running AICA system -- the measuring tool of the
setup roadmap (`docs/uebersicht/fahrplan-aufbau.md`).

It only subscribes and reads parameters. It never publishes, never sets a
parameter and never triggers a lifecycle transition.

Runs INSIDE the AICA container, as user ``ros2`` (FastDDS hands the shared
memory segments to their owner -- as root the topics are visible but no data
arrives) and with a set ROS_DOMAIN_ID (an empty one crashes rclpy/CLI)::

    docker cp signal_reader.py <c>:/tmp/ && docker cp ../../roboter_tetris/contracts.py <c>:/tmp/
    docker exec -u ros2 -e ROS_DOMAIN_ID=0 <c> bash -lc 'python3 /tmp/signal_reader.py cameras'

``contracts.py`` is taken from the installed package if present, otherwise
from the directory of this script -- copying it along makes the tool work with
an old image too.

Commands
--------
topics                      all topics with their types
cameras                     RealSense nodes: serial, global_time, auto exposure
stamps     --topic T        ros_now - header.stamp: offset, drift (ms/s), rate
cartesian                   flange pose (robot_state_broadcaster), median/sigma
joints                      joint positions and velocities, median/sigma
objects | tracks | target | not_pickable | picked_id |
follower_status | world_state | object_position
                            decode a contract signal (S1-S10)

Common options: ``--duration`` (s), ``--topic`` (default: the one topic whose
name ends in ``/<command>``), ``--live`` (one line per message, throttled by
``--period``), ``--csv PATH`` (raw rows), ``--summary`` (last line is a JSON
summary for `b23_compare.py`).
"""

import argparse
import json
import math
import os
import statistics
import sys
import time

try:
    from roboter_tetris import contracts as C
except Exception:  # old image or package not importable -- use the copy
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import contracts as C  # noqa: E402

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray


ROBOT_STATE_PREFIX = "/hardware/robot_state_broadcaster/"

#: name -> (unpack function, list attribute or None)
CONTRACT_SIGNALS = {
    "objects": (C.unpack_objects, "objects"),
    "tracks": (C.unpack_tracks, "tracks"),
    "world_state": (C.unpack_world_state, "entries"),
    "target": (C.unpack_target, None),
    "not_pickable": (C.unpack_not_pickable, None),
    "picked_id": (C.unpack_picked_id, None),
    "follower_status": (C.unpack_follower_status, None),
    "object_position": (C.unpack_object_position, None),
}

STATE_NAMES = {v: k for k, v in vars(C).items() if k.startswith("STATE_")}
OUTCOME_NAMES = {v: k for k, v in vars(C).items() if k.startswith("OUTCOME_")}


# -- helpers ------------------------------------------------------------------

def med_sd(values):
    values = [v for v in values if v is not None and not math.isnan(v)]
    if not values:
        return float("nan"), float("nan")
    sd = statistics.pstdev(values) if len(values) > 1 else 0.0
    return statistics.median(values), sd


def resolve_topic(node, wanted, suffix):
    if wanted:
        return wanted
    deadline = time.time() + 2.0
    names = []
    while time.time() < deadline:
        names = [n for n, _ in node.get_topic_names_and_types()]
        hits = [n for n in names if n.endswith("/" + suffix)]
        if len(hits) == 1:
            return hits[0]
        if len(hits) > 1:
            sys.exit(f"Mehrere Topics enden auf /{suffix}: {hits} -- bitte --topic angeben")
        rclpy.spin_once(node, timeout_sec=0.2)
    sys.exit(f"Kein Topic endet auf /{suffix}. Vorhanden: "
             + ", ".join(sorted(n for n in names if not n.startswith("/realsense"))))


def spin_for(node, duration, stop=lambda: False):
    end = time.time() + duration
    while time.time() < end and not stop():
        rclpy.spin_once(node, timeout_sec=0.1)


class Csv:
    def __init__(self, path):
        self.f = open(path, "w") if path else None

    def row(self, *values):
        if self.f:
            self.f.write(",".join(f"{v:.6f}" if isinstance(v, float) else str(v)
                                  for v in values) + "\n")

    def close(self):
        if self.f:
            self.f.close()


# -- commands -----------------------------------------------------------------

def cmd_topics(node, args):
    spin_for(node, 1.0)
    for name, types in sorted(node.get_topic_names_and_types()):
        print(f"{name:70s} {', '.join(types)}")


def cmd_cameras(node, args):
    from rcl_interfaces.srv import GetParameters
    spin_for(node, 1.0)
    params = ["serial_no", "device_type",
              "rgb_camera.global_time_enabled", "depth_module.global_time_enabled",
              "rgb_camera.enable_auto_exposure", "rgb_camera.exposure",
              "rgb_camera.profile", "rgb_camera.color_profile",
              "depth_module.profile", "depth_module.depth_profile",
              "align_depth.enable"]
    known = {"f1370107": "BASISKAMERA (L515)", "241122074842": "ROBOTERKAMERA (D435i)"}
    # AICA hosts the camera drivers so that they need not appear in the node
    # list -- derive the node from the topic prefix of its camera_info instead.
    cams = sorted({name.rsplit("/", 1)[0] for name, _ in node.get_topic_names_and_types()
                   if name.endswith("/color_camera_info")})
    if not cams:
        print("Keine Kameraknoten gefunden.")
    for cam in cams:
        cli = node.create_client(GetParameters, cam + "/get_parameters")
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f"{cam}: kein Parameterdienst")
            continue
        # One request per parameter: the driver answers a request that names a
        # single undeclared parameter with an empty list for ALL of them.
        vals = {}
        for name in params:
            fut = cli.call_async(GetParameters.Request(names=[name]))
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            if not fut.done() or fut.result() is None or not fut.result().values:
                continue
            v = fut.result().values[0]
            val = {1: v.bool_value, 2: v.integer_value, 3: v.double_value,
                   4: v.string_value}.get(v.type)
            if val is not None:
                vals[name] = val
        if "serial_no" not in vals:
            print(f"{cam}: kein serial_no gemeldet -- gelesen: {sorted(vals)}")
            continue
        serial = str(vals["serial_no"]).strip('"').lstrip("_")
        print(f"\n{cam}  ->  {known.get(serial, 'unbekannter Serial')}  (serial {serial})")
        for name in params[1:]:
            if name in vals:
                flag = ""
                if name.endswith("global_time_enabled") and vals[name] is False:
                    flag = "   <-- MUSS true sein (Einrichtung §1)"
                if (name.endswith("enable_auto_exposure") and vals[name] is True
                        and serial == "241122074842"):
                    flag = "   <-- bei der Roboterkamera aus (Einrichtung §1)"
                print(f"   {name:36s} {vals[name]}{flag}")


def cmd_stamps(node, args):
    from rosidl_runtime_py.utilities import get_message
    topic = args.topic
    if not topic:
        sys.exit("stamps braucht --topic (z. B. .../color_camera_info)")
    spin_for(node, 1.0)
    types = dict(node.get_topic_names_and_types()).get(topic)
    if not types:
        sys.exit(f"Topic {topic} nicht gefunden")
    samples = []  # (ros_now, stamp)

    def cb(msg):
        now = node.get_clock().now().nanoseconds * 1e-9
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        samples.append((now, st))

    node.create_subscription(get_message(types[0]), topic, cb, qos_profile_sensor_data)
    spin_for(node, args.duration)
    if len(samples) < 3:
        sys.exit(f"Nur {len(samples)} Nachrichten -- läuft der Stream? (als ros2 gestartet?)")
    t0 = samples[0][0]
    xs = [s[0] - t0 for s in samples]
    off = [(s[0] - s[1]) * 1000.0 for s in samples]
    n = len(xs)
    mx, my = sum(xs) / n, sum(off) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, off)) / sxx if sxx else 0.0
    stamps = [s[1] for s in samples]
    frozen = sum(1 for a, b in zip(stamps, stamps[1:]) if b <= a)
    print(f"{topic}")
    print(f"  Nachrichten     {n} in {xs[-1]:.1f} s  ->  {(n - 1) / xs[-1]:.2f} Hz")
    print(f"  Versatz         Anfang {off[0]:+.1f} ms, Ende {off[-1]:+.1f} ms, Median {statistics.median(off):+.1f} ms")
    print(f"  Drift           {slope:+.3f} ms/s   (gut: |Drift| < 0,1; L515 ohne global_time: ~ +4)")
    print(f"  Stempel         {frozen} nicht steigend" + ("   <-- EINGEFROREN?" if frozen else ""))
    epoch_years = abs(statistics.median(off)) / 1000.0 / 3.15e7
    if epoch_years > 0.5:
        print(f"  ⚠ Versatz ~{epoch_years:.1f} Jahre -- Stempel in fremder Epoche (Hardwareuhr)")


def _decode_state(msg):
    import clproto
    return clproto.decode(bytes(msg.data))


def cmd_cartesian(node, args):
    from modulo_interfaces.msg import EncodedState
    topic = args.topic or ROBOT_STATE_PREFIX + "cartesian_state"
    rows, csv = [], Csv(args.csv)

    def cb(msg):
        st = _decode_state(msg)
        p, q = st.get_position(), st.get_orientation()
        r = (p[0], p[1], p[2], q.w, q.x, q.y, q.z)
        rows.append(r)
        csv.row(time.time(), *r)
        if args.live:
            _throttled(args, f"x {p[0]*1000:8.2f}  y {p[1]*1000:8.2f}  z {p[2]*1000:8.2f} mm")

    node.create_subscription(EncodedState, topic, cb, 10)
    spin_for(node, args.duration)
    csv.close()
    if not rows:
        sys.exit("Keine Daten (als ros2 gestartet? Hardware aktiv?)")
    labels = ["x", "y", "z", "qw", "qx", "qy", "qz"]
    stats = [med_sd([r[i] for r in rows]) for i in range(7)]
    print(f"{topic}   n = {len(rows)}")
    for (m, s), l in zip(stats, labels):
        unit = 1000.0 if l in "xyz" else 1.0
        print(f"  {l:3s} median {m*unit: 11.4f}   sigma {s*unit: .4f}" + ("  mm" if unit > 1 else ""))
    qw, qx, qy, qz = (stats[i][0] for i in range(3, 7))
    tool_z = (2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -tool_z[2]))))
    print(f"  Werkzeugachse weicht {tilt:.2f}° von der Lotrechten (nach unten) ab")
    frozen = len(rows) > 20 and all(stats[i][1] == 0.0 for i in range(7))
    if frozen:
        print("  ⚠ Streuung exakt null -- Treiber speist keine frischen Daten (Pendant-Programm gestoppt?)")
    if args.summary:
        print(json.dumps({"x": stats[0][0] * 1000, "y": stats[1][0] * 1000, "z": stats[2][0] * 1000,
                          "sx": stats[0][1] * 1000, "sy": stats[1][1] * 1000, "sz": stats[2][1] * 1000,
                          "tilt_deg": tilt, "n": len(rows), "frozen": frozen}))


def cmd_joints(node, args):
    from modulo_interfaces.msg import EncodedState
    topic = args.topic or ROBOT_STATE_PREFIX + "joint_state"
    names, pos, vel, csv = [], [], [], Csv(args.csv)

    def cb(msg):
        st = _decode_state(msg)
        if not names:
            names.extend(st.get_names())
        p, v = list(st.get_positions()), list(st.get_velocities())
        pos.append(p)
        vel.append(v)
        csv.row(time.time(), *p, *v)
        if args.live:
            _throttled(args, "  ".join(f"{math.degrees(a):7.1f}" for a in p)
                       + "  |v|max " + f"{max(abs(x) for x in v):.3f} rad/s")

    node.create_subscription(EncodedState, topic, cb, 10)
    spin_for(node, args.duration)
    csv.close()
    if not pos:
        sys.exit("Keine Daten")
    print(f"{topic}   n = {len(pos)}")
    for i, nm in enumerate(names):
        m, s = med_sd([p[i] for p in pos])
        vmax = max(abs(v[i]) for v in vel)
        print(f"  {nm:24s} {math.degrees(m):9.2f}°  sigma {math.degrees(s):.3f}°  |v|max {vmax:.3f} rad/s")
    w2 = [n for n in names if "wrist_2" in n]
    if w2:
        i = names.index(w2[0])
        d = min(abs(math.degrees(p[i])) % 180.0 for p in pos)
        d = min(d, 180.0 - d)
        print(f"  Handgelenk-Singularität: wrist_2 kam ihr bis auf {d:.1f}° nahe (0°/±180°)")


_last_print = [0.0]


def _throttled(args, line):
    now = time.time()
    if now - _last_print[0] >= args.period:
        _last_print[0] = now
        print(f"{time.strftime('%H:%M:%S')}  {line}", flush=True)


def _fmt_entry(e):
    status = ""
    if hasattr(e, "status"):
        status = f" st {int(e.status)}"
    extra = ""
    if hasattr(e, "vx"):
        extra = f" v ({e.vx*1000:6.1f},{e.vy*1000:6.1f}) mm/s"
    if hasattr(e, "present"):
        extra += f" pr {int(e.present)} pk {int(e.picked)} oob {int(e.out_of_bounds)}"
    return (f"[id {int(e.id)}{status} x {e.x*1000:7.1f} y {e.y*1000:7.1f} "
            f"z {e.z*1000:6.1f} h {e.height*1000:5.1f}{extra}]")


def cmd_contract(node, args):
    unpack, list_attr = CONTRACT_SIGNALS[args.command]
    topic = resolve_topic(node, args.topic, args.command)
    csv, msgs, errors = Csv(args.csv), [], [0]
    last_seq = [None]

    def cb(msg):
        try:
            m = unpack(msg.data)
        except C.ContractError as exc:
            errors[0] += 1
            if errors[0] <= 3:
                print(f"VERTRAGSFEHLER: {exc}", flush=True)
            return
        if m is None:
            return
        msgs.append((time.time(), m))
        csv.row(time.time(), *msg.data)
        if args.command == "picked_id":
            if m.seq != last_seq[0]:
                last_seq[0] = m.seq
                print(f"{time.strftime('%H:%M:%S')}  seq {int(m.seq)}  id {int(m.id)}  "
                      f"outcome {int(m.outcome)} {OUTCOME_NAMES.get(int(m.outcome), '')}", flush=True)
            return
        if not args.live:
            return
        if list_attr:
            head = f"n {len(getattr(m, list_attr))}"
            if hasattr(m, "n_pool"):
                head += f"  pool {int(m.n_pool)} v ({m.v_belt_x*1000:.1f},{m.v_belt_y*1000:.1f}) mm/s"
            elif hasattr(m, "v_belt"):
                head += f"  v_band {m.v_belt*1000:.1f} mm/s"
            _throttled(args, head + "  " + " ".join(_fmt_entry(e) for e in getattr(m, list_attr)))
        elif args.command == "follower_status":
            _throttled(args, f"{STATE_NAMES.get(int(m.state), m.state):15s} id {int(m.target_id)}  "
                             f"err laengs {m.err_long*1000:7.1f} quer {m.err_lat*1000:7.1f} "
                             f"z {m.err_z*1000:7.1f} mm  w {m.w_effective:.2f}")
        elif args.command == "target":
            if m.has_target:
                _throttled(args, f"ZIEL id {int(m.id)} x {m.x*1000:.1f} y {m.y*1000:.1f} "
                                 f"t_rest {m.t_rest:.2f} s  zone_up {m.zone_upstream:.3f}  "
                                 f"ebene {m.grasp_plane:.3f}  v ({m.vx*1000:.1f},{m.vy*1000:.1f})")
            else:
                _throttled(args, f"kein Ziel   zone_up {m.zone_upstream:.3f}  ebene {m.grasp_plane:.3f}")
        else:
            _throttled(args, str(m))

    node.create_subscription(Float64MultiArray, topic, cb, 10)
    print(f"lese {topic} für {args.duration:.0f} s ...", flush=True)
    spin_for(node, args.duration)
    csv.close()
    print(f"\n{topic}: {len(msgs)} Nachrichten, {errors[0]} Vertragsfehler")
    if len(msgs) > 1:
        ts = [m[1].t for m in msgs if hasattr(m[1], "t")]
        if ts:
            new = sum(1 for a, b in zip(ts, ts[1:]) if b != a)
            span = msgs[-1][0] - msgs[0][0]
            print(f"  {len(msgs)/span:.1f} Nachr./s, davon {new/span:.1f}/s mit neuem t")
    if list_attr and msgs:
        _summarise_entries(args, [m for _, m in msgs], list_attr)


def _summarise_entries(args, msgs, list_attr):
    by_id = {}
    for m in msgs:
        for e in getattr(m, list_attr):
            by_id.setdefault(int(e.id), []).append(e)
    if not by_id:
        print("  keine Einträge")
        return
    print(f"  {'id':>4} {'n':>5} {'x mm':>9} {'σx':>5} {'y mm':>9} {'σy':>5} "
          f"{'z mm':>7} {'h mm':>6} {'σh':>4} {'l mm':>6} {'b mm':>6}  status")
    for oid, es in sorted(by_id.items()):
        (x, sx), (y, sy) = med_sd([e.x for e in es]), med_sd([e.y for e in es])
        (z, _), (h, sh) = med_sd([e.z for e in es]), med_sd([e.height for e in es])
        (ln, _), (wd, _) = med_sd([e.length for e in es]), med_sd([e.width for e in es])
        st = ""
        if hasattr(es[0], "status"):
            seq = []
            for e in es:
                if not seq or seq[-1] != int(e.status):
                    seq.append(int(e.status))
            st = "->".join(map(str, seq))
        print(f"  {oid:4d} {len(es):5d} {x*1000:9.1f} {sx*1000:5.2f} {y*1000:9.1f} {sy*1000:5.2f} "
              f"{z*1000:7.1f} {h*1000:6.1f} {sh*1000:4.1f} {ln*1000:6.1f} {wd*1000:6.1f}  {st}")
    if hasattr(msgs[-1], "n_pool"):
        m = msgs[-1]
        print(f"  Pool am Ende: n_pool {int(m.n_pool)}, v = ({m.v_belt_x*1000:.2f}, "
              f"{m.v_belt_y*1000:.2f}) mm/s, |v| = {math.hypot(m.v_belt_x, m.v_belt_y)*1000:.2f} mm/s")
    if args.summary:
        oid, es = max(by_id.items(), key=lambda kv: len(kv[1]))
        (x, sx), (y, sy) = med_sd([e.x for e in es]), med_sd([e.y for e in es])
        (z, _), (h, sh) = med_sd([e.z for e in es]), med_sd([e.height for e in es])
        print(json.dumps({"id": oid, "n": len(es), "x": x * 1000, "y": y * 1000, "z": z * 1000,
                          "h": h * 1000, "sx": sx * 1000, "sy": sy * 1000, "sh": sh * 1000,
                          "ids_seen": len(by_id)}))


# -- main ---------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("command", choices=["topics", "cameras", "stamps", "cartesian", "joints"]
                   + sorted(CONTRACT_SIGNALS))
    p.add_argument("--topic")
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--live", action="store_true")
    p.add_argument("--period", type=float, default=0.5, help="Mindestabstand der Live-Zeilen (s)")
    p.add_argument("--csv")
    p.add_argument("--summary", action="store_true")
    args = p.parse_args()

    rclpy.init()
    node = Node("signal_reader_" + str(os.getpid()))
    try:
        {"topics": cmd_topics, "cameras": cmd_cameras, "stamps": cmd_stamps,
         "cartesian": cmd_cartesian, "joints": cmd_joints}.get(args.command, cmd_contract)(node, args)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
