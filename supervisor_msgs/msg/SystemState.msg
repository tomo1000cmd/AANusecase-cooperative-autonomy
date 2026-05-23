# System-wide state for supervisory control
string robot_id
string[] active_tasks
string system_mode       # normal, degraded, emergency, local_only
float64 system_health
int32 num_active_agents
string[] failed_agents
builtin_interfaces/Time timestamp
