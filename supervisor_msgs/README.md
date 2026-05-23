# supervisor_msgs Package

Message and service definitions for the Distributed Supervisory Control Layer.

## Messages

### AgentStatus
Published by each agent to report its operational state.

```
string agent_id              # e.g., "voa_1" 
string agent_type            # voa, mla, ca, fa
string state                 # idle, active, error, recovering
float64 health               # 0.0 (dead) to 1.0 (perfect)
string task_id               # current task being executed
string[] capabilities        # list of supported operations
builtin_interfaces/Time timestamp
```

### SystemState  
Published by supervisor to report overall system health.

```
string robot_id              # robot identifier
string[] active_tasks        # currently executing task IDs
string system_mode           # normal, degraded, emergency, local_only
float64 system_health        # average health of all agents
int32 num_active_agents      # count of healthy agents
string[] failed_agents       # agents with health < threshold
builtin_interfaces/Time timestamp
```

### TaskAllocation
Published by supervisor to assign tasks to agents.

```
string task_id               # unique task identifier
string target_agent          # agent to execute task
string task_type             # navigation, monitoring, forecasting, etc
string priority              # critical, high, medium, low
string[] parameters          # task-specific parameters
builtin_interfaces/Time deadline
```

### ConflictResolution
Published by supervisor when agent conflicts are detected/resolved.

```
string robot_id              # robot identifier
string[] conflicting_agents  # agents involved in conflict
string[] conflicting_tasks   # tasks in conflict
string resolution_rule       # which policy rule was applied
string recommended_action    # action taken by supervisor
```

## Services

### AllocateTask
Agents call this service to request task execution approval.

**Request:**
```
string task_id               # unique task identifier
string agent_id              # requesting agent
string task_type             # type of task
string priority              # critical, high, medium, low
string[] parameters          # task parameters
```

**Response:**
```
bool success                 # true if allocation was approved
string allocation_id         # identifier for this allocation
string rejection_reason      # if not approved, why
```

### QueryAgentStatus
Query supervisor for information about a specific agent.

**Request:**
```
string agent_id              # agent to query
string query_type            # capabilities, state, utilization, available_slots
```

**Response:**
```
string response_status       # success, not_found, error
string[] agent_info          # query-dependent response data
```

---

**Usage**: Import in your agent nodes:
```python
from supervisor_msgs.msg import AgentStatus, SystemState, TaskAllocation, ConflictResolution
from supervisor_msgs.srv import AllocateTask, QueryAgentStatus
```

