"""Public scaffold metadata for package-level tests."""

EXPECTED_ACTIONS = (
    "TakeoffSwarm",
    "MoveLeader",
    "ChangeFormation",
    "PauseSwarm",
    "LandSwarm",
)

EXPECTED_MESSAGES = (
    "LeaderGoal",
    "FormationMode",
    "MissionCommand",
    "FailsafeCommand",
    "VehicleSetpoint",
    "VehicleStatus",
)

SWARM_NAMESPACE = "/swarm"
VEHICLE_NAMESPACES = ("/vehicle_1", "/vehicle_2", "/vehicle_3")
