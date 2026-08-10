# Status summary 隔離 raw PX4 telemetry，保護 swarm logic 不依賴 px4_msgs shape。
builtin_interfaces/Time stamp
uint8 vehicle_id
string role
string px4_namespace
string slot
float64 x
float64 y
float64 z
float64 yaw
float64 vx
float64 vy
float64 vz
bool armed
string nav_state
bool offboard_available
float32 last_telemetry_age_sec
string vehicle_state
