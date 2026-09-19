#!/usr/bin/env python3
"""Can traffic control actually reach the simulator? A three-level probe.

Reviewer R3 objects that RoadSeed has no traffic control, so signal-caused
crashes (a large share of the NHTSA pre-crash classes) are approximated "with
geometry and motion". The chain that would fix this has three links, and only
the third is in doubt:

  L1  OpenDRIVE can express it.  The 1.5M schema has <signals><signal/>
      (xsd/OpenDRIVE_1.5M.xsd, t_road_signals_signal) with <validity
      fromLane toLane> to bind a signal to the lanes it governs, and
      dynamic="yes" to mark a light rather than a static sign. Our generator
      library already emits it (scenariogeneration.xodr.Signal +
      Road.add_signal). --inject proves this end offline: it injects signals
      into a generated map and XSD-validates the result.

  L2  CARLA instantiates it.  Whether client.generate_opendrive_world() turns
      <signal> records into traffic.traffic_light actors on a procedurally
      generated map is UNVERIFIED -- our map caches never recorded a traffic
      light count. --carla answers it by counting actors.

  L3  The ADS can perceive it.  This is the link that decides whether the test
      is valid, and it is strictly harder than L2: an actor can exist without
      CARLA associating it with the junction, and our agents read the signal
      through that association (runner/src/data_collector.py uses
      hero.get_traffic_light()). If the association is missing, the light spawns
      but the ADS is still blind to it, so the scenario still does not test
      signal compliance. --carla checks this by placing a vehicle on the
      approach leg and asking it for its traffic light.

L1 runs anywhere. L2/L3 need a CARLA server, so run --carla on the CARLA host.

    # locally: inject + validate
    uv run python tools/probe_traffic_signal.py --inject \
        outputs/medoid_runs/122_Waymo_February_18_2024/map.xodr \
        --out /tmp/122_signalled.xodr

    # on the CARLA host
    python tools/probe_traffic_signal.py --carla /tmp/122_signalled.xodr \
        --host 127.0.0.1 --port 2000
"""
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
XODR_XSD = ROOT / "xsd" / "OpenDRIVE_1.5M.xsd"

# OpenDRIVE signal type codes. CARLA maps these to actor blueprints in
# LibCarla road::SignalType; 1000001 is the three-state traffic light in the
# German catalogue, 206/205 are stop/yield. Confirm against the CARLA build in
# use -- if L2 reports zero actors, a wrong type code is the first suspect.
SIGNAL_TYPES = {
    "traffic_light": {"type": "1000001", "subtype": "-1", "dynamic": "yes",
                      "name": "trafficLight", "height": "3.0", "width": "0.4",
                      "zOffset": "4.5"},
    "stop_sign": {"type": "206", "subtype": "-1", "dynamic": "no",
                  "name": "stopSign", "height": "0.8", "width": "0.8",
                  "zOffset": "2.0"},
    "yield_sign": {"type": "205", "subtype": "-1", "dynamic": "no",
                   "name": "yieldSign", "height": "0.8", "width": "0.8",
                   "zOffset": "2.0"},
}


def approach_legs(root: ET.Element) -> list[ET.Element]:
    """Non-junction roads whose successor is a junction: the approach legs.

    Our junction templates always attach legs by successor (verified on the
    generated 4-leg maps), so s = road length is the stop-line end.
    """
    legs = []
    for road in root.findall("road"):
        if road.get("junction") not in (None, "-1"):
            continue
        suc = road.find("link/successor")
        if suc is not None and suc.get("elementType") == "junction":
            legs.append(road)
    return legs


def driving_lane_ids(road: ET.Element, side: str) -> list[int]:
    ls = road.find("lanes/laneSection")
    if ls is None:
        return []
    return [int(ln.get("id")) for ln in ls.findall(f"{side}/lane")
            if ln.get("type") == "driving"]


def inject(xodr_path: Path, out_path: Path, kind: str,
           setback: float) -> tuple[bool, str]:
    """Put one signal at the stop line of every approach leg, governing the
    lanes that travel toward the junction."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    legs = approach_legs(root)
    if not legs:
        return False, "no approach legs found (is this a junction map?)"

    spec = SIGNAL_TYPES[kind]
    added = 0
    for road in legs:
        length = float(road.get("length"))
        s = max(0.0, length - setback)
        # Lanes travelling in +s (toward the junction) are the right-hand set.
        lanes = sorted(driving_lane_ids(road, "right"))
        if not lanes:
            continue
        signals = road.find("signals")
        if signals is None:
            # <signals> comes after <lanes> in the t_road sequence.
            signals = ET.Element("signals")
            children = list(road)
            lanes_el = road.find("lanes")
            road.insert(children.index(lanes_el) + 1 if lanes_el is not None
                        else len(children), signals)
        sig = ET.SubElement(signals, "signal", {
            "s": f"{s:.3f}",
            "t": "-6.0",          # right shoulder, outside the driving lanes
            "id": f"{road.get('id')}001",
            "name": spec["name"],
            "dynamic": spec["dynamic"],
            "orientation": "+",   # governs traffic travelling in +s
            "zOffset": spec["zOffset"],
            "country": "DEU",
            "type": spec["type"],
            "subtype": spec["subtype"],
            "height": spec["height"],
            "width": spec["width"],
        })
        # <validity> binds the signal to the lanes it governs -- this is how a
        # signal can be red for the NPC's leg and green for the ego's.
        ET.SubElement(sig, "validity", {"fromLane": str(min(lanes)),
                                        "toLane": str(max(lanes))})
        added += 1

    ET.indent(tree, space="    ")
    out_path.write_text(ET.tostring(root, encoding="unicode",
                                   xml_declaration=True))
    print(f"[L1] injected {added} {kind} signal(s) into {added} approach leg(s)")
    print(f"[L1] wrote {out_path}")

    try:
        import xmlschema
    except ImportError:
        return True, f"added {added}; xmlschema not installed, skipped validation"
    schema = xmlschema.XMLSchema(str(XODR_XSD))
    errs = list(schema.iter_errors(out_path.read_text()))
    if errs:
        print(f"[L1] XSD INVALID: {len(errs)} violation(s)")
        for e in errs[:3]:
            print(f"       {str(e.reason or e)[:160]}")
        return False, f"XSD invalid ({len(errs)} violations)"
    print("[L1] XSD valid")
    return True, f"added {added} signal(s), XSD valid"


def probe_carla(xodr_path: Path, host: str, port: int, timeout: float,
                settle_ticks: int) -> tuple[bool, str]:
    import carla  # noqa: PLC0415 -- only available on the CARLA host

    xodr_text = xodr_path.read_text()
    n_signals = xodr_text.count("<signal ")
    print(f"[L2] map declares {n_signals} <signal> record(s)")

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    params = carla.OpendriveGenerationParameters(
        vertex_distance=2.0, max_road_length=500.0, wall_height=0.0,
        additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=True)
    world = client.generate_opendrive_world(xodr_text, params)
    cmap = world.get_map()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    for _ in range(settle_ticks):
        world.tick()

    lights = list(world.get_actors().filter("traffic.traffic_light*"))
    signs = [a for a in world.get_actors()
             if a.type_id.startswith("traffic.") and "traffic_light" not in a.type_id]
    print(f"[L2] CARLA spawned {len(lights)} traffic.traffic_light actor(s), "
          f"{len(signs)} other traffic.* actor(s)")
    for a in signs[:5]:
        print(f"       {a.type_id}")
    if not lights:
        print("[L2] FAIL: generate_opendrive_world did not instantiate the signals.")
        print("       -> check the signal type code against this CARLA build's")
        print("          road::SignalType before concluding it is unsupported.")
        return False, f"0 traffic lights from {n_signals} <signal> records"

    # L3: does the association the ADS relies on exist?
    bp = world.get_blueprint_library().filter("vehicle.*")[0]
    spawned = None
    associated = None
    for wp in cmap.generate_waypoints(4.0):
        if wp.is_junction:
            continue
        # a waypoint whose next few metres enter a junction == an approach lane
        nxt = wp.next(12.0)
        if not any(w.is_junction for w in nxt):
            continue
        tf = wp.transform
        tf.location.z += 0.5
        spawned = world.try_spawn_actor(bp, tf)
        if spawned is None:
            continue
        for _ in range(5):
            world.tick()
        associated = spawned.get_traffic_light()
        if associated is not None:
            break
        spawned.destroy()
        spawned = None

    try:
        if associated is None:
            print("[L3] FAIL: a vehicle on the approach lane got no traffic light "
                  "from get_traffic_light().")
            print("       The actors exist but CARLA did not bind them to the "
                  "junction, so the ADS cannot perceive the signal and the "
                  "scenario still does not test signal compliance.")
            return False, f"{len(lights)} lights spawned but no junction association"

        state = spawned.get_traffic_light_state()
        print(f"[L3] vehicle is governed by traffic light id={associated.id}, "
              f"state={state}")
        associated.set_state(carla.TrafficLightState.Red)
        associated.freeze(True)
        world.tick()
        after = spawned.get_traffic_light_state()
        print(f"[L3] after forcing Red, the vehicle reads: {after}")
        ok = str(after) == str(carla.TrafficLightState.Red)
        print(f"[L3] {'PASS' if ok else 'FAIL'}: signal state is "
              f"{'controllable and visible to the ego' if ok else 'not settable'}")
        return ok, (f"{len(lights)} lights, association OK, "
                    f"state {'controllable' if ok else 'not settable'}")
    finally:
        if spawned is not None:
            spawned.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--inject", type=Path, metavar="XODR",
                    help="L1: add signals to this generated map and XSD-validate")
    ap.add_argument("--out", type=Path, help="output path for --inject")
    ap.add_argument("--kind", default="traffic_light", choices=sorted(SIGNAL_TYPES))
    ap.add_argument("--setback", type=float, default=0.5,
                    help="metres before the junction end to place the signal")
    ap.add_argument("--carla", type=Path, metavar="XODR",
                    help="L2/L3: load this map in CARLA and check the signals")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--settle-ticks", type=int, default=20)
    args = ap.parse_args()

    if not args.inject and not args.carla:
        ap.error("pass --inject and/or --carla")

    verdicts: list[tuple[str, bool, str]] = []
    if args.inject:
        out = args.out or args.inject.with_name(args.inject.stem + "_signalled.xodr")
        ok, msg = inject(args.inject, out, args.kind, args.setback)
        verdicts.append(("L1 OpenDRIVE emit", ok, msg))
    if args.carla:
        try:
            ok, msg = probe_carla(args.carla, args.host, args.port,
                                  args.timeout, args.settle_ticks)
        except ImportError:
            print("[L2] carla module unavailable -- run this on the CARLA host")
            return 3
        except Exception as exc:  # noqa: BLE001
            print(f"[L2] probe crashed: {type(exc).__name__}: {exc}")
            verdicts.append(("L2/L3 CARLA", False, f"{type(exc).__name__}: {exc}"))
        else:
            verdicts.append(("L2/L3 CARLA", ok, msg))

    print("\n=== verdict ===")
    for name, ok, msg in verdicts:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {msg}")
    return 0 if all(ok for _, ok, _ in verdicts) else 1


if __name__ == "__main__":
    raise SystemExit(main())
