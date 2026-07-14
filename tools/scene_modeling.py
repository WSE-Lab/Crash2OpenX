#!/usr/bin/env python3
"""Scene metamodel & UML generation (MODELS-style modeling layer).

Promotes scene_seed.json (synthesis output, OpenSCENARIO-flavored) into an
explicit domain metamodel with actor roles, scenario categories, inter-actor
relations (incl. collision pairing + traffic control), and phase-decomposed
behaviors. Emits PlantUML + Mermaid:
  - metamodel class diagram     (M2 — the modeling language)
  - scenario instance object    (M1 — this specific scene)
  - activity diagram            (behavioral view — phase chain per actor)

Read-only over the existing pipeline: consumes scene_seed.json and writes
artifacts to <run_dir>/modeling/, never mutates seeds, xodr, or xosc.

CLI:
    python tools/scene_modeling.py outputs/web_runs/<name>/scene_seed.json
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


# ============================================================
# Metamodel (M2)  — types are documented as string enums so the class diagram
# stays readable; runtime validation is intentionally light (synthesis already
# enforces the source schema in tools/api_infer_scene_seed_v2.py).
# ============================================================

@dataclass
class Trigger:
    type: str            # sim_time | longitudinal_distance | cartesian_distance | ttc | traveled_distance
    value: float
    on_entity: str | None = None


@dataclass
class Action:
    type: str            # maintain_speed | decelerate_to | accelerate_to | lane_change | cross_road | walk_along | stop | follow_path
    params: dict[str, Any] = field(default_factory=dict)
    trigger: Trigger | None = None


@dataclass
class Position:
    type: str            # relative_lane | world | junction_leg
    reference: str | None = None
    lane_offset: int = 0
    long_offset: float = 0.0
    lat_offset: float = 0.0
    leg: str | None = None


@dataclass
class Actor:
    id: str
    role: str            # ego | target | interferer | background | infrastructure
    category: str        # vehicle | pedestrian | cyclist | bus | traffic_signal | stop_sign | yield_sign
    initial_position: Position | None = None
    initial_speed: float | None = None


@dataclass
class Relation:
    type: str            # lead_follow | oncoming | parallel | crossing | roadside | collision | yields_to
    from_actor: str
    to_actor: str


@dataclass
class Phase:
    id: str
    actor_id: str
    action: Action
    order: int = 0       # order within the actor's behavior chain


@dataclass
class EnvironmentCondition:
    time_of_day: str = ""
    weather: str = ""
    friction_scale: float = 1.0


@dataclass
class Scenario:
    name: str
    type: str            # car_following | cut_in | rear_collision | oncoming_conflict | lane_change | vru_crossing | junction_crossing | junction_turn | static_obstacle | unknown
    sut_maneuver: str = "straight"
    control: str = "none"
    actors: list[Actor] = field(default_factory=list)
    relations: list[Relation] = field(default_factory=list)
    phases: list[Phase] = field(default_factory=list)
    environment: EnvironmentCondition = field(default_factory=EnvironmentCondition)


# ============================================================
# Inference: scene_seed -> Scenario (M1 instance)
# ============================================================

def _all_params(npc: dict) -> dict:
    return {**((npc.get("behavior") or {}).get("params") or {}), **(npc.get("params") or {})}


def _to_speed(v) -> float | None:
    """Speed field is sometimes ``null`` in scene_seed (LLM omitted it). Treat
    null / non-numeric / 0 as 'unspecified' rather than crashing on float(None)."""
    if isinstance(v, (int, float)) and v:
        return float(v)
    return None


def _infer_position(npc: dict) -> Position:
    pos = npc.get("position")
    side = npc.get("side", "none")
    p = _all_params(npc)
    if pos == "ahead_same_lane":
        return Position("relative_lane", "ego", 0, float(p.get("gap", 15.0)), 0.0)
    if pos == "behind_same_lane":
        return Position("relative_lane", "ego", 0, -float(p.get("gap", 15.0)), 0.0)
    if pos == "adjacent":
        return Position("relative_lane", "ego",
                        1 if side == "left" else -1,
                        float(p.get("long", 5.0)), 0.0)
    if pos == "oncoming":
        return Position("relative_lane", "ego", 2, float(p.get("gap", 15.0)), 0.0)
    if pos == "roadside":
        lat = float(p.get("lateral", 3.5)) * (1 if side == "left" else -1)
        return Position("relative_lane", "ego", 0, float(p.get("gap", 15.0)), lat)
    if pos in ("cross", "opposing_leg"):
        return Position("junction_leg", "ego", leg=pos)
    return Position("relative_lane", "ego")


def _infer_scenario_type(scene: dict) -> str:
    blocks = [(n.get("behavior") or {}).get("block") for n in scene.get("npcs", [])]
    positions = [n.get("position") for n in scene.get("npcs", [])]
    # priority: junction > cut_in > VRU > car-following > rear / oncoming > static
    if "junction_cross" in blocks:
        return "junction_crossing"
    if "junction_turn" in blocks:
        return "junction_turn"
    if "cut_in" in blocks:
        return "cut_in"
    if "front_brake" in blocks:
        return "car_following"
    if "rear_hit" in blocks:
        return "rear_collision"
    if "oncoming" in blocks:
        return "oncoming_conflict"
    if any(b in ("cross", "walk_along") for b in blocks):
        return "vru_crossing"
    if any(b in ("stopped_ahead", "static_block") for b in blocks):
        return "static_obstacle"
    if any(p in ("cross", "opposing_leg") for p in positions):
        return "junction_crossing"
    return "unknown"


def _infer_relation_from_position(npc: dict) -> Relation | None:
    pos = npc.get("position")
    if pos == "ahead_same_lane":
        return Relation("lead_follow", from_actor=npc["id"], to_actor="ego")
    if pos == "behind_same_lane":
        return Relation("lead_follow", from_actor="ego", to_actor=npc["id"])
    if pos == "adjacent":
        return Relation("parallel", from_actor="ego", to_actor=npc["id"])
    if pos == "oncoming":
        return Relation("oncoming", from_actor="ego", to_actor=npc["id"])
    if pos == "roadside":
        return Relation("roadside", from_actor=npc["id"], to_actor="ego")
    if pos in ("cross", "opposing_leg"):
        return Relation("crossing", from_actor=npc["id"], to_actor="ego")
    return None


def _infer_role(npc_id: str, scene: dict) -> str:
    """Use collision pairing as primary signal: whoever is paired with ego in
    collision is the *target*. Other movers are *interferers*; stationary
    obstacles are *background*."""
    coll = scene.get("collision") or {}
    a, b = coll.get("a"), coll.get("b")
    if npc_id in (a, b) and ("ego" in (a, b)):
        return "target"
    # background = static / stopped
    for n in scene.get("npcs", []):
        if n["id"] != npc_id:
            continue
        block = (n.get("behavior") or {}).get("block")
        if block in ("stopped_ahead", "static_block"):
            return "background"
        break
    return "interferer"


def _phases_for_npc(npc: dict, start_order: int) -> list[Phase]:
    """Decompose an NPC behavior block into one or more Phase entries — this
    is what makes activity diagrams non-trivial. Each phase is one (actor,
    action) and gets a sequential ``order`` within the actor's chain."""
    nid = npc["id"]
    block = (npc.get("behavior") or {}).get("block")
    p = _all_params(npc)
    speed = float(p.get("speed", 8.0))
    out: list[Phase] = []

    def add(name: str, action: Action) -> None:
        out.append(Phase(id=f"{nid}_{name}", actor_id=nid, action=action,
                         order=start_order + len(out)))

    if block == "front_brake":
        add("cruise", Action("maintain_speed", {"speed": speed},
                             Trigger("sim_time", 0.0)))
        add("brake", Action("decelerate_to",
                            {"target_speed": float(p.get("end_speed", 0.0)),
                             "duration": float(p.get("brake_t", 1.2))},
                            Trigger("longitudinal_distance",
                                    float(p.get("trig_dist", 18.0)),
                                    on_entity=nid)))
    elif block in ("rear_hit", "oncoming"):
        add("travel", Action("maintain_speed", {"speed": speed},
                             Trigger("sim_time", 0.0)))
    elif block == "cut_in":
        add("cruise", Action("maintain_speed", {"speed": speed},
                             Trigger("sim_time", 0.0)))
        add("lane_change", Action("lane_change", {"target": "ego_lane"},
                                  Trigger("ttc", float(p.get("trig_ttc", 3.0)),
                                          on_entity=nid)))
    elif block == "cross":
        add("wait", Action("stop", {}, Trigger("sim_time", 0.0)))
        add("cross", Action("cross_road", {"speed": float(p.get("speed", 1.5))},
                            Trigger("cartesian_distance",
                                    float(p.get("trig_dist", 15.0)),
                                    on_entity=nid)))
    elif block == "walk_along":
        add("walk", Action("walk_along", {"speed": float(p.get("speed", 1.5))},
                           Trigger("sim_time", 0.0)))
    elif block in ("stopped_ahead", "static_block"):
        add("stationary", Action("stop", {}, Trigger("sim_time", 0.0)))
    elif block in ("junction_cross", "junction_turn"):
        add("approach", Action("maintain_speed", {"speed": speed},
                               Trigger("sim_time", 0.0)))
        add("traverse", Action("follow_path", {"path": "junction_route"},
                               Trigger("ttc", float(p.get("trig_ttc", 3.0)),
                                       on_entity=nid)))
    return out


_CONTROL_TO_INFRA = {
    "traffic_light": ("traffic_light_1", "traffic_signal"),
    "stop_sign":     ("stop_sign_1",     "stop_sign"),
    "yield":         ("yield_sign_1",    "yield_sign"),
}


def _infer_causal_relations(scene: dict, npcs: list[dict],
                            phases: list[Phase]) -> list[Relation]:
    """Higher-order relations derived from existing scene + phase facts.

    These complement the geometric relations (lead_follow/oncoming/...) with
    *causal*, *contention*, and *temporal* structure that MODELS-style behavior
    modeling cares about. All three are pure derivations — no new info added
    beyond what scene_seed + phase decomposition already encode.

    - ``causes(npc, counterpart)``    NPC's active block is the proximate cause
                                       of the collision counterpart's reaction
                                       (NPC ∈ collision pair, block ∈ active set)
    - ``competes_for(a, b)``           a and b contend for the same lane /
                                       intersection space (cut_in / VRU vs ego,
                                       or multiple junction movers pairwise)
    - ``precedes(ego, npc)``           ego must be moving before any NPC whose
                                       phase has a spatial trigger can fire —
                                       trigger.on_entity ≠ ∅ implies the NPC's
                                       reactive phase depends on ego's state
    """
    out: list[Relation] = []

    # ---- causes: NPC active behavior -> collision counterpart ----
    coll = scene.get("collision") or {}
    coll_a, coll_b = coll.get("a"), coll.get("b")
    active_blocks = {"front_brake", "cut_in", "cross", "walk_along",
                     "rear_hit", "oncoming", "junction_cross", "junction_turn"}
    for n in npcs:
        nid = n["id"]
        block = (n.get("behavior") or {}).get("block")
        if block not in active_blocks:
            continue
        if nid not in (coll_a, coll_b):
            continue
        other = coll_a if coll_b == nid else coll_b
        if other and other != nid:
            out.append(Relation("causes", from_actor=nid, to_actor=other))

    # ---- competes_for: lane / intersection contention ----
    lane_competitors: list[str] = []
    junction_competitors: list[str] = []
    for n in npcs:
        nid = n["id"]
        block = (n.get("behavior") or {}).get("block")
        position = n.get("position")
        if block in ("cut_in", "cross", "walk_along"):
            lane_competitors.append(nid)
        if block in ("junction_cross", "junction_turn") or \
                position in ("cross", "opposing_leg"):
            junction_competitors.append(nid)
    for nid in lane_competitors:
        out.append(Relation("competes_for", from_actor=nid, to_actor="ego"))
    for i, a in enumerate(junction_competitors):
        out.append(Relation("competes_for", from_actor=a, to_actor="ego"))
        for b in junction_competitors[i + 1:]:
            out.append(Relation("competes_for", from_actor=a, to_actor=b))

    # ---- precedes: ego -> any NPC with a spatial trigger ----
    spatial = {"longitudinal_distance", "cartesian_distance",
               "ttc", "traveled_distance"}
    reactive_npcs: set[str] = set()
    for ph in phases:
        if ph.actor_id == "ego":
            continue
        if ph.action.trigger and ph.action.trigger.type in spatial:
            reactive_npcs.add(ph.actor_id)
    for nid in sorted(reactive_npcs):
        out.append(Relation("precedes", from_actor="ego", to_actor=nid))

    return out


def infer_scene_model(scene_seed: dict, *, name: str = "scenario") -> Scenario:
    scene = scene_seed.get("scene") or scene_seed
    if not isinstance(scene, dict):
        scene = {}
    npcs = scene.get("npcs") or []
    sut = scene.get("sut") or {}
    env = scene.get("environment") or {}
    control = scene.get("control") or "none"

    scenario_type = _infer_scenario_type(scene)

    actors: list[Actor] = [
        Actor(id="ego", role="ego", category="vehicle",
              initial_speed=_to_speed((sut.get("params") or {}).get("speed"))),
    ]
    relations: list[Relation] = []
    phases: list[Phase] = []
    next_order = 0

    # Ego always has a "drive to goal" phase (mirrors osc_blocks' AcquirePosition).
    phases.append(Phase(
        id="ego_drive",
        actor_id="ego",
        action=Action("follow_path", {"goal": "scenario_destination",
                                       "maneuver": sut.get("maneuver", "straight")},
                       Trigger("sim_time", 0.0)),
        order=0,
    ))

    for npc in npcs:
        nid = npc["id"]
        role = _infer_role(nid, scene)
        actors.append(Actor(
            id=nid,
            role=role,
            category=npc.get("kind", "vehicle"),
            initial_position=_infer_position(npc),
            initial_speed=_to_speed(_all_params(npc).get("speed")),
        ))
        rel = _infer_relation_from_position(npc)
        if rel is not None:
            relations.append(rel)
        npc_phs = _phases_for_npc(npc, start_order=0)
        phases.extend(npc_phs)

    # Collision pair as a first-class relation.
    coll = scene.get("collision") or {}
    if coll.get("a") and coll.get("b"):
        relations.append(Relation("collision",
                                   from_actor=coll["a"], to_actor=coll["b"]))

    # Traffic control as an infrastructure actor + yields_to relation.
    if control in _CONTROL_TO_INFRA:
        infra_id, infra_cat = _CONTROL_TO_INFRA[control]
        actors.append(Actor(id=infra_id, role="infrastructure", category=infra_cat))
        # Every mover yields to / is governed by the control.
        for a in actors:
            if a.role in ("ego", "target", "interferer"):
                relations.append(Relation("yields_to",
                                           from_actor=a.id, to_actor=infra_id))

    # Higher-order relations: causes / competes_for / precedes (option A).
    relations.extend(_infer_causal_relations(scene, npcs, phases))

    return Scenario(
        name=name,
        type=scenario_type,
        sut_maneuver=sut.get("maneuver", "straight"),
        control=control,
        actors=actors,
        relations=relations,
        phases=phases,
        environment=EnvironmentCondition(
            time_of_day=str(env.get("time_of_day", "")),
            weather=str(env.get("weather", "")),
            friction_scale=float(env.get("friction_scale", 1.0) or 1.0),
        ),
    )


# ============================================================
# Rendering — PlantUML
# ============================================================

def to_plantuml_class() -> str:
    """Metamodel class diagram (M2). Independent of any specific scenario."""
    return """@startuml
title Road Scene Metamodel (M2)
hide empty members
skinparam classAttributeIconSize 0

class Scenario {
  +name : String
  +type : ScenarioType
  +sutManeuver : Maneuver
  +control : TrafficControl
}
enum ScenarioType {
  CAR_FOLLOWING
  CUT_IN
  REAR_COLLISION
  ONCOMING_CONFLICT
  LANE_CHANGE
  VRU_CROSSING
  JUNCTION_CROSSING
  JUNCTION_TURN
  STATIC_OBSTACLE
  UNKNOWN
}
enum Maneuver { STRAIGHT LEFT RIGHT }
enum TrafficControl { NONE TRAFFIC_LIGHT STOP_SIGN YIELD UNKNOWN }

class Actor {
  +id : String
  +role : ActorRole
  +category : ActorCategory
  +initialSpeed : Double
}
enum ActorRole { EGO TARGET INTERFERER BACKGROUND INFRASTRUCTURE }
enum ActorCategory {
  VEHICLE PEDESTRIAN CYCLIST BUS
  TRAFFIC_SIGNAL STOP_SIGN YIELD_SIGN
}

class Position {
  +type : PositionKind
  +laneOffset : Integer
  +longOffset : Double
  +latOffset : Double
  +leg : String
}
enum PositionKind { RELATIVE_LANE WORLD JUNCTION_LEG }

class Relation {
  +type : RelationType
}
enum RelationType {
  LEAD_FOLLOW ONCOMING PARALLEL CROSSING
  ROADSIDE COLLISION YIELDS_TO
  CAUSES COMPETES_FOR PRECEDES
}

class Phase {
  +id : String
  +order : Integer
}

class Action {
  +type : ActionType
  +params : Map
}
enum ActionType {
  MAINTAIN_SPEED DECELERATE_TO ACCELERATE_TO LANE_CHANGE
  CROSS_ROAD WALK_ALONG STOP FOLLOW_PATH
}

class Trigger {
  +type : TriggerType
  +value : Double
}
enum TriggerType {
  SIM_TIME LONGITUDINAL_DISTANCE CARTESIAN_DISTANCE
  TTC TRAVELED_DISTANCE
}

class EnvironmentCondition {
  +timeOfDay : String
  +weather : String
  +frictionScale : Double
}

Scenario "1" *-- "1..*" Actor : participants
Scenario "1" *-- "0..*" Relation : relations
Scenario "1" *-- "1..*" Phase : phases
Scenario "1" *-- "1"    EnvironmentCondition

Actor "1" *-- "0..1" Position : initialPosition
Position "*" --> "0..1" Actor : reference

Relation "*" --> "1" Actor : from
Relation "*" --> "1" Actor : to

Phase "*" --> "1" Actor : performedBy
Phase "1" *-- "1" Action
Phase "0..1" --> "0..1" Phase : next

Action "1" *-- "0..1" Trigger : guard

@enduml
"""


def _puml_id(s: str) -> str:
    return "".join(c if c.isalnum() else "_" for c in s)


def to_plantuml_instance(sc: Scenario) -> str:
    L = ["@startuml", f"title Scenario Instance (M1): {sc.name} — {sc.type}",
         "hide empty members", "skinparam objectAttributeIconSize 0"]
    L.append('object "scenario : Scenario" as Scen {')
    L.append(f"  type = {sc.type}")
    L.append(f"  sutManeuver = {sc.sut_maneuver}")
    L.append(f"  control = {sc.control}")
    L.append("}")

    for a in sc.actors:
        aid = _puml_id(a.id)
        L.append(f'object "{a.id} : Actor" as A_{aid} {{')
        L.append(f"  role = {a.role}")
        L.append(f"  category = {a.category}")
        if a.initial_speed is not None:
            L.append(f"  initialSpeed = {a.initial_speed}")
        L.append("}")
        L.append(f"Scen *-- A_{aid}")
        if a.initial_position is not None:
            pos = a.initial_position
            L.append(f'object "pos_{a.id} : Position" as P_{aid} {{')
            L.append(f"  type = {pos.type}")
            if pos.reference: L.append(f"  reference = {pos.reference}")
            if pos.lane_offset: L.append(f"  laneOffset = {pos.lane_offset}")
            if pos.long_offset: L.append(f"  longOffset = {pos.long_offset}")
            if pos.lat_offset: L.append(f"  latOffset = {pos.lat_offset}")
            if pos.leg: L.append(f"  leg = {pos.leg}")
            L.append("}")
            L.append(f"A_{aid} *-- P_{aid}")

    for i, r in enumerate(sc.relations):
        rid = f"R{i}"
        L.append(f'object "{r.type} : Relation" as {rid} {{')
        L.append(f"  type = {r.type}")
        L.append("}")
        L.append(f"Scen *-- {rid}")
        L.append(f"{rid} --> A_{_puml_id(r.from_actor)} : from")
        L.append(f"{rid} --> A_{_puml_id(r.to_actor)} : to")

    e = sc.environment
    L.append('object "env : EnvironmentCondition" as Env {')
    L.append(f"  timeOfDay = {e.time_of_day or '(unspec)'}")
    L.append(f"  weather = {e.weather or '(unspec)'}")
    L.append(f"  frictionScale = {e.friction_scale}")
    L.append("}")
    L.append("Scen *-- Env")
    L.append("@enduml")
    return "\n".join(L)


def _trigger_label(t: Trigger | None) -> str:
    if t is None:
        return ""
    sym = {"sim_time": "t",
           "longitudinal_distance": "d_long",
           "cartesian_distance": "d_xy",
           "ttc": "ttc",
           "traveled_distance": "travel"}.get(t.type, t.type)
    return f" [{sym} < {t.value}]" if t.type != "sim_time" else f" [t ≥ {t.value}s]"


def to_plantuml_activity(sc: Scenario) -> str:
    """Activity diagram — one swimlane per actor, ordered phase chain, parallel."""
    L = ["@startuml", f"title Scenario Behavior: {sc.name} ({sc.type})",
         "skinparam defaultFontName Helvetica"]
    # Group phases by actor, preserve order.
    by_actor: dict[str, list[Phase]] = {}
    for ph in sc.phases:
        by_actor.setdefault(ph.actor_id, []).append(ph)
    for aid in by_actor:
        by_actor[aid].sort(key=lambda p: p.order)

    if not by_actor:
        L += ["start", ":empty;", "stop", "@enduml"]
        return "\n".join(L)

    L.append("start")
    L.append("fork")
    first = True
    for aid, phs in by_actor.items():
        if not first:
            L.append("fork again")
        first = False
        L.append(f"|{aid}|")
        for ph in phs:
            params = ", ".join(f"{k}={v}" for k, v in ph.action.params.items())
            trig = _trigger_label(ph.action.trigger)
            label = f"{ph.action.type}({params}){trig}"
            L.append(f":{label};")
    L.append("end fork")
    L.append("stop")
    L.append("@enduml")
    return "\n".join(L)


# ============================================================
# Rendering — Mermaid (for web/markdown viewing)
# ============================================================

def to_mermaid_class() -> str:
    return """classDiagram
    direction TB
    class Scenario {
      +String name
      +ScenarioType type
      +Maneuver sutManeuver
      +TrafficControl control
    }
    class Actor {
      +String id
      +ActorRole role
      +ActorCategory category
      +Double initialSpeed
    }
    class Position {
      +PositionKind type
      +Integer laneOffset
      +Double longOffset
      +Double latOffset
    }
    class Relation {
      +RelationType type
    }
    class Phase {
      +String id
      +Integer order
    }
    class Action {
      +ActionType type
      +Map params
    }
    class Trigger {
      +TriggerType type
      +Double value
    }
    class EnvironmentCondition {
      +String timeOfDay
      +String weather
      +Double frictionScale
    }
    Scenario "1" *-- "1..*" Actor
    Scenario "1" *-- "0..*" Relation
    Scenario "1" *-- "1..*" Phase
    Scenario "1" *-- "1" EnvironmentCondition
    Actor "1" *-- "0..1" Position
    Relation "*" --> "1" Actor : from
    Relation "*" --> "1" Actor : to
    Phase "*" --> "1" Actor : performedBy
    Phase "1" *-- "1" Action
    Phase "0..1" --> "0..1" Phase : next
    Action "1" *-- "0..1" Trigger
"""


def to_mermaid_activity(sc: Scenario) -> str:
    by_actor: dict[str, list[Phase]] = {}
    for ph in sc.phases:
        by_actor.setdefault(ph.actor_id, []).append(ph)
    for aid in by_actor:
        by_actor[aid].sort(key=lambda p: p.order)

    L = ["flowchart TB", f"  Start([Start: {sc.type}])", "  Fork{{fork}}",
         "  Start --> Fork"]
    end_nodes: list[str] = []
    for aid, phs in by_actor.items():
        safe = _puml_id(aid)
        L.append(f"  subgraph LANE_{safe} [{aid}]")
        prev = "Fork"
        for i, ph in enumerate(phs):
            node = f"N_{safe}_{i}"
            params = ", ".join(f"{k}={v}" for k, v in ph.action.params.items())
            trig = _trigger_label(ph.action.trigger).strip()
            lbl = f"{ph.action.type}<br/>{params}"
            if trig:
                lbl += f"<br/>{trig}"
            L.append(f'    {node}["{lbl}"]')
            L.append(f"    {prev} --> {node}")
            prev = node
        L.append("  end")
        end_nodes.append(prev)
    L.append("  Join{{join}}")
    for n in end_nodes:
        L.append(f"  {n} --> Join")
    L.append("  Join --> End([End])")
    return "\n".join(L)


# ============================================================
# CLI
# ============================================================


def to_mermaid_instance(sc: Scenario) -> str:
    """Return a Mermaid graph TD showing actors as nodes and relations as labeled edges.
    Suited for rendering the M1 instance of a specific scenario in the web UI."""
    L = ["graph TD"]
    role_ico = {"ego": "🟢", "target": "🚗", "lead": "🚙",
                "obstacle": "🟥", "pedestrian": "🚶", "cyclist": "🚴"}
    for a in sc.actors:
        ico = role_ico.get(a.role, "🚗")
        # Compose tooltip-like label across two lines.
        line2 = f"{a.role} · {a.category}"
        pos_str = ""
        if a.initial_position:
            p = a.initial_position
            if p.type == "relative_lane":
                pos_str = f" · {p.reference}{('+' + str(p.long_offset)) if (p.long_offset or 0) >= 0 else str(p.long_offset)}m"
            elif p.type == "absolute":
                pos_str = f" · absolute"
            elif p.type == "leg":
                pos_str = f" · leg {p.leg or '?'}"
        safe = _puml_id(a.id)
        L.append(f'  A_{safe}["{ico} <b>{a.id}</b><br/>{line2}{pos_str}"]')
    # Relations as edges
    for r in sc.relations:
        f = _puml_id(r.from_actor)
        t = _puml_id(r.to_actor)
        if r.type == "collision":
            L.append(f'  A_{f} ==>|💥 collision| A_{t}')
        elif r.type == "causes":
            L.append(f'  A_{f} -.->|causes| A_{t}')
        elif r.type == "precedes":
            L.append(f'  A_{f} -.->|precedes| A_{t}')
        else:
            L.append(f'  A_{f} -->|{r.type}| A_{t}')
    L.append("  classDef ego fill:#0e2618,stroke:#00d4aa,color:#e6e9f0;")
    L.append("  classDef npc fill:#1a2138,stroke:#2a3756,color:#e6e9f0;")
    for a in sc.actors:
        safe = _puml_id(a.id)
        cls = "ego" if a.role == "ego" else "npc"
        L.append(f"  class A_{safe} {cls};")
    return "\n".join(L) + "\n"


def scenario_to_dict(sc: Scenario) -> dict:
    return asdict(sc)


def render_all(scene_seed: dict, name: str) -> tuple[Scenario, dict[str, str]]:
    sc = infer_scene_model(scene_seed, name=name)
    artifacts = {
        "scene_model.json": json.dumps(scenario_to_dict(sc), ensure_ascii=False, indent=2) + "\n",
        "metamodel_class.puml": to_plantuml_class(),
        "instance.puml": to_plantuml_instance(sc),
        "activity.puml": to_plantuml_activity(sc),
        "metamodel_class.mmd": to_mermaid_class(),
        "activity.mmd": to_mermaid_activity(sc),
        "instance.mmd": to_mermaid_instance(sc),
    }
    return sc, artifacts


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Promote scene_seed.json into a typed scene metamodel + UML diagrams.")
    ap.add_argument("scene_seed", type=Path, help="path to scene_seed.json")
    ap.add_argument("--out-dir", type=Path, default=None,
                    help="output dir (default: <scene_seed parent>/modeling/)")
    ap.add_argument("--name", default=None,
                    help="scenario name (default: parent dir name)")
    args = ap.parse_args()

    seed_path = args.scene_seed
    if not seed_path.is_absolute():
        seed_path = (Path.cwd() / seed_path).resolve()
    if not seed_path.exists():
        print(f"error: scene_seed not found: {seed_path}", file=sys.stderr)
        return 2

    scene_seed = json.loads(seed_path.read_text(encoding="utf-8"))
    name = args.name or seed_path.parent.name
    out_dir = args.out_dir or (seed_path.parent / "modeling")
    out_dir.mkdir(parents=True, exist_ok=True)

    sc, artifacts = render_all(scene_seed, name)
    for fname, text in artifacts.items():
        (out_dir / fname).write_text(text, encoding="utf-8")

    print(json.dumps({
        "scene_seed": str(seed_path),
        "out_dir": str(out_dir),
        "scenario_type": sc.type,
        "actor_count": len(sc.actors),
        "actors": [{"id": a.id, "role": a.role, "category": a.category} for a in sc.actors],
        "relation_count": len(sc.relations),
        "relations": [{"type": r.type, "from": r.from_actor, "to": r.to_actor} for r in sc.relations],
        "phase_count": len(sc.phases),
        "artifacts": list(artifacts),
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
