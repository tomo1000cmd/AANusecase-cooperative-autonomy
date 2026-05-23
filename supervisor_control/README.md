# Distributed Supervisory Control Layer

## Overview

The **Supervisory Control Layer** is a distributed, rule-based coordination middleware for multi-agent robotic systems. Each robot runs its own instance, enabling autonomous operation if centralized coordination fails.

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│            Supervisory Control Node (per robot)         │
│  ┌────────────────────────────────────────────────────┐ │
│  │  Task Allocation Engine                            │ │
│  │  - Service Server: AllocateTask                    │ │
│  │  - Strategy: least_loaded, round_robin, priority   │ │
│  └────────────────────────────────────────────────────┘ │
│                                                          │
│  ┌────────────────────────────────────────────────────┐ │
│  │  Conflict Resolution Engine                         │ │
│  │  - Rule-based resolution (priority, health)        │ │
│  │  - Publishes ConflictResolution messages           │ │
│  └────────────────────────────────────────────────────┘ │
│                                                          │
│  ┌────────────────────────────────────────────────────┐ │
│  │  System Health Monitor                              │ │
│  │  - Tracks agent states (idle, active, error)       │ │
│  │  - Detects heartbeat timeouts                      │ │
│  │  - Publishes SystemState messages                  │ │
│  └────────────────────────────────────────────────────┘ │
│                                                          │
│  ┌────────────────────────────────────────────────────┐ │
│  │  Policy Engine (Rule-Based)                         │ │
│  │  - Loads YAML policy configuration                 │ │
│  │  - Agent priorities, health thresholds             │ │
│  │  - System degradation modes                        │ │
│  └────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
          ↓              ↓              ↓              ↓
       ┌─────┐        ┌─────┐        ┌─────┐        ┌─────┐
       │ VOA │        │ MLA │        │ CA  │        │ FA  │
       └─────┘        └─────┘        └─────┘        └─────┘
```

## System Modes

| Mode       | Condition                          | Behavior                              |
|------------|-----------------------------------|---------------------------------------|
| **NORMAL** | All agents healthy                  | Centralized coordination active       |
| **DEGRADED** | 50-80% agents healthy           | Limited coordination, priority-based  |
| **EMERGENCY** | < 30% agents healthy            | Minimal coordination, health-aware    |
| **LOCAL_ONLY** | Centralized comm timeout         | No central coordination; independent  |

## Key Components

### 1. **Agent Status Monitoring**
- Subscription to `{robot_id}/agents/+/status` topics
- Tracks: agent state, health (0.0-1.0), capabilities, current task
- Automatic detection of failed agents (heartbeat timeout: 5.0s)

### 2. **Task Allocation Service**
- ROS2 Service: `{robot_id}/supervisor/allocate_task`
- Validates agent availability, health, and workload
- Returns allocation success/rejection with reason

### 3. **System State Publishing**
- Topic: `{robot_id}/supervisor/system_state`
- Publishes every 1.0 second
- Contains: system mode, active tasks, failed agents, overall health

### 4. **Conflict Resolution**
- Rule-based resolution using agent priorities
- Publishes `ConflictResolution` messages on conflict
- Supports preemption based on priority/health

### 5. **Policy Configuration**
- YAML-based rule definitions (`.config/policies.yaml`)
- Customizable priorities per agent type
- Configurable health thresholds and timeouts

## Messages

### AgentStatus
Agent sends periodic status updates:
```
string agent_id          # e.g., "voa_1"
string agent_type        # voa, mla, ca, fa
string state             # idle, active, error, recovering
float64 health           # 0.0 - 1.0
string task_id           # current task
string[] capabilities    # e.g., ["navigation", "control"]
```

### SystemState
Supervisor publishes system state:
```
string robot_id
string[] active_tasks
string system_mode       # normal, degraded, emergency, local_only
float64 system_health
int32 num_active_agents
string[] failed_agents
```

### TaskAllocation
Supervisor publishes task assignments:
```
string task_id
string target_agent
string task_type
string priority          # critical, high, medium, low
string[] parameters
```

## Services

### AllocateTask (Service)
Request:
```
string task_id
string agent_id
string task_type
string priority
string[] parameters
```
Response:
```
bool success
string allocation_id
string rejection_reason
```

### QueryAgentStatus (Service)
Request:
```
string agent_id
string query_type        # capabilities, state, utilization, available_slots
```
Response:
```
string response_status   # success, not_found
string[] agent_info
```

## Usage

### Launch Supervisory Control for a Robot
```bash
ros2 launch supervisor_control supervisor_control.launch.py robot_id:=robot_1
```

### Query Agent Status
```bash
ros2 service call /robot_1/supervisor/query_agent supervisor_msgs/srv/QueryAgentStatus \
  "{agent_id: voa_1, query_type: capabilities}"
```

### Allocate a Task
```bash
ros2 service call /robot_1/supervisor/allocate_task supervisor_msgs/srv/AllocateTask \
  "{task_id: task_001, agent_id: voa_1, task_type: navigation, priority: high, parameters: [param1, param2]}"
```

### Monitor System State
```bash
ros2 topic echo /robot_1/supervisor/system_state
```

## Policy Configuration

Edit `config/policies.yaml` to customize:

- **Task allocation strategy**: least_loaded, round_robin, capability_match, priority_based
- **Agent priorities**: VOA=1 (highest), MLA=2, CA=3, FA=4
- **Maximum tasks per agent**: Prevent overload
- **Health thresholds**: Define degradation points
- **Conflict resolution rules**: Priority override, health-based reassignment
- **Heartbeat timeout**: Time to consider agent offline (default: 5.0s)

## Resilience Features

1. **Automatic Failover to Local-Only Mode**
   - No manual intervention needed
   - Triggered when centralized coordination times out
   - Agents continue independently

2. **Health-Based Decision Making**
   - Low-health agents get fewer tasks
   - Critical agents (VOA) prioritized during degradation
   - Automatic task reassignment on health decline

3. **Conflict Detection & Resolution**
   - Detects competing task requests
   - Applies priority-based resolution rules
   - Publishes conflict notifications for visibility

4. **Graceful Degradation**
   - System enters DEGRADED mode at 50% agent health
   - EMERGENCY mode at 30% health
   - LOCAL_ONLY when centralized communication fails

## Integration with Agents

Each agent (VOA, MLA, CA, FA) should:

1. **Publish AgentStatus regularly** (1-2 Hz)
   ```python
   agent_status_msg = AgentStatus()
   agent_status_msg.agent_id = "voa_1"
   agent_status_msg.agent_type = "voa"
   agent_status_msg.state = "active"
   agent_status_msg.health = 0.95
   agent_status_msg.task_id = "task_001"
   agent_status_pub.publish(agent_status_msg)
   ```

2. **Query supervisor before executing tasks** (optional)
   ```python
   future = allocate_task_client.call_async(AllocateTask.Request(...))
   ```

3. **Listen for task allocation messages** (optional)
   ```python
   self.create_subscription(TaskAllocation, f'{robot_id}/supervisor/task_allocation', 
                           self.task_callback, qos_profile)
   ```

## Testing

### Monitor System Health
```bash
# Terminal 1: Start supervisor
ros2 launch supervisor_control supervisor_control.launch.py robot_id:=robot_1

# Terminal 2: Publish agent status
ros2 topic pub /robot_1/agents/voa_1/status supervisor_msgs/msg/AgentStatus \
  "{agent_id: voa_1, agent_type: voa, state: active, health: 0.95, task_id: '', capabilities: [navigation]}"

# Terminal 3: Monitor system state
ros2 topic echo /robot_1/supervisor/system_state
```

## Extending the Architecture

### Adding New Agents
1. Define agent type in `policies.yaml`
2. Agent publishes `AgentStatus` messages
3. Supervisor automatically discovers and manages it

### Custom Conflict Resolution
Modify `resolve_conflicts()` method in `supervisory_control_node.py` to implement custom strategies

### Dynamic Policy Updates
Add service to reload policy configuration without restarting supervisor

---

**Author**: AANusecase Team  
**License**: Apache License 2.0
