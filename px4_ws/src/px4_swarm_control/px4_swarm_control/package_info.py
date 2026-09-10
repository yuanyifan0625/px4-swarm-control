"""Public scaffold metadata for package-level tests."""

EXPECTED_ACTIONS = (
    "ArmSwarm",
    "TakeoffSwarm",
    "MoveLeader",
    "ChangeFormation",
    "PauseSwarm",
    "LandSwarm",
)

EXPECTED_MESSAGES = (
    "LeaderGoal",
    "ManualJog",
    "ManualJogStatus",
    "FormationMode",
    "MissionCommand",
    "FailsafeCommand",
    "VehicleSetpoint",
    "VehicleStatus",
)

SWARM_NAMESPACE = "/swarm"
VEHICLE_NAMESPACES = ("/MAV1", "/MAV2", "/MAV3")
