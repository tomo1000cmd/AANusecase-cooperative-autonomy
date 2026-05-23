# Agent operational status message
string agent_id
string agent_type  # voa, mla, ca, fa
string state       # idle, active, error, recovering
float64 health     # 0.0 - 1.0 (health metric)
string task_id
string[] capabilities
builtin_interfaces/Time timestamp
