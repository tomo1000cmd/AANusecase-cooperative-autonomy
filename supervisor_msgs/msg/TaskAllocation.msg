# Task allocation request to agents
string task_id
string target_agent    # agent_id to allocate to
string task_type       # e.g., navigation, coverage, monitoring, forecasting
string priority        # low, medium, high, critical
string[] parameters    # task-specific parameters
builtin_interfaces/Time deadline
