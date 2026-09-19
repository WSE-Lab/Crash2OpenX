"""Semantic vehicle classes mapped to verified CARLA 0.9.16 blueprints.

The model selects a source-described class, never a simulator asset name.
Dimensions in OpenSCENARIO remain catalog metadata; CARLA loads each named
blueprint's actual mesh, collision shape and dynamics.
"""

VEHICLE_CLASSES = {
    "car": ("vehicle.tesla.model3", "car"),
    "suv": ("vehicle.nissan.patrol_2021", "car"),
    "pickup": ("vehicle.tesla.cybertruck", "car"),
    "van": ("vehicle.mercedes.sprinter", "van"),
    "truck": ("vehicle.carlamotors.european_hgv", "truck"),
    "bus": ("vehicle.mitsubishi.fusorosa", "bus"),
    "ambulance": ("vehicle.ford.ambulance", "van"),
}

PARKED_HEADINGS = {"parallel", "opposite", "perpendicular"}


def actor_attributes(actor):
    """Validate and preserve source-level attributes during normalization."""
    attributes = {}
    if "vehicle_class" in actor:
        value = actor["vehicle_class"]
        if actor.get("kind") != "vehicle" or value not in VEHICLE_CLASSES:
            raise ValueError(f"invalid vehicle_class {value!r} for {actor.get('kind')!r}")
        attributes["vehicle_class"] = value
    if "parked_heading" in actor:
        value = actor["parked_heading"]
        if value not in PARKED_HEADINGS:
            raise ValueError(f"invalid parked_heading {value!r}")
        attributes["parked_heading"] = value
    return attributes
