#!/usr/bin/env python3
"""
Distributed Supervisory Control Layer
Provides task allocation, conflict resolution, and system state management
for multi-agent coordination (VOA, MLA, CA, FA).

Each robot runs its own instance for resilience—can operate independently
if centralized coordination fails.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from supervisor_msgs.msg import AgentStatus, SystemState, TaskAllocation, ConflictResolution
from supervisor_msgs.srv import QueryAgentStatus, AllocateTask
from std_msgs.msg import String
import json
import yaml
import time
from pathlib import Path
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple
from datetime import datetime


class SystemMode(Enum):
    """System operation modes"""
    NORMAL = "normal"              # All agents operational, centralized coordination
    DEGRADED = "degraded"          # Some agents failed, limited coordination
    EMERGENCY = "emergency"        # Multiple agents failed, minimal coordination
    LOCAL_ONLY = "local_only"      # No centralized coordination, agents act independently


@dataclass
class AgentInfo:
    """Agent state tracking"""
    agent_id: str
    agent_type: str
    state: str
    health: float
    task_id: str
    capabilities: List[str]
    last_update: float


class SupervisoryControlNode(Node):
    """
    Distributed supervisory control layer for multi-agent systems.
    Runs on each robot and coordinates local agents (VOA, MLA, CA, FA).
    """

    def __init__(self):
        super().__init__('supervisor_control')
        self.get_logger().info("Initializing Supervisory Control Node...")

        # ========== Parameters ==========
        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('policy_file', 'policies.yaml')
        self.declare_parameter('health_threshold', 0.3)
        self.declare_parameter('centralized_timeout', 5.0)  # seconds
        
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        policy_file = self.get_parameter('policy_file').get_parameter_value().string_value
        self.health_threshold = self.get_parameter('health_threshold').get_parameter_value().double_value
        self.centralized_timeout = self.get_parameter('centralized_timeout').get_parameter_value().double_value

        # ========== State Management ==========
        self.agents: Dict[str, AgentInfo] = {}
        self.active_tasks: Dict[str, str] = {}  # task_id -> agent_id
        self.system_mode = SystemMode.NORMAL
        self.last_heartbeat_time = time.time()
        self.policies = {}

        # ========== Load Configuration ==========
        self.load_policies(policy_file)

        # ========== Publishers ==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.system_state_pub = self.create_publisher(
            SystemState, f'{self.robot_id}/supervisor/system_state', qos_profile)
        self.conflict_pub = self.create_publisher(
            ConflictResolution, f'{self.robot_id}/supervisor/conflicts', qos_profile)
        self.task_alloc_pub = self.create_publisher(
            TaskAllocation, f'{self.robot_id}/supervisor/task_allocation', qos_profile)

        # ========== Subscribers ==========
        # Subscribe to each known agent type (VOA, MLA, CA, FA)
        for agent_type in ['voa', 'mla', 'ca', 'fa']:
            self.create_subscription(
                AgentStatus, f'{self.robot_id}/{agent_type}/status',
                self.agent_status_callback, qos_profile)
            self.get_logger().debug(f"Subscribed to {self.robot_id}/{agent_type}/status")

        # ========== Service Servers ==========
        self.alloc_service = self.create_service(
            AllocateTask, f'{self.robot_id}/supervisor/allocate_task',
            self.allocate_task_callback)
        
        self.query_service = self.create_service(
            QueryAgentStatus, f'{self.robot_id}/supervisor/query_agent',
            self.query_agent_status_callback)

        # ========== Timers ==========
        self.create_timer(0.5, self.monitor_system_health)
        self.create_timer(1.0, self.publish_system_state)
        self.create_timer(1.0, self.check_centralized_communication)

        self.get_logger().info(f"✓ Supervisory Control initialized for {self.robot_id}")

    def load_policies(self, policy_file: str) -> None:
        """Load decision policies from configuration file."""
        try:
            # Try to find policy file in package share directory or current directory
            from ament_index_python.packages import get_package_share_directory
            package_share = get_package_share_directory('supervisor_control')
            policy_path = Path(package_share) / 'config' / policy_file
            
            if not policy_path.exists():
                policy_path = Path(policy_file)
            
            if policy_path.exists():
                with open(policy_path, 'r') as f:
                    self.policies = yaml.safe_load(f) or {}
                self.get_logger().info(f"✓ Loaded policies from {policy_path}")
            else:
                self.get_logger().warn(f"Policy file not found: {policy_path}, using defaults")
                self.policies = self.get_default_policies()
        except Exception as e:
            self.get_logger().error(f"Failed to load policies: {e}")
            self.policies = self.get_default_policies()

    def get_default_policies(self) -> Dict:
        """Return default control policies."""
        return {
            'task_allocation': {
                'strategy': 'least_loaded',
                'priority_levels': ['critical', 'high', 'medium', 'low']
            },
            'conflict_resolution': {
                'priority_override': True,
                'rules': [
                    {'condition': 'higher_priority', 'action': 'preempt_lower'},
                    {'condition': 'agent_health_low', 'action': 'reassign_task'},
                    {'condition': 'mutual_exclusion', 'action': 'time_share'}
                ]
            },
            'system_degradation': {
                'health_threshold': self.health_threshold,
                'degraded_threshold': 0.5,
                'emergency_threshold': 0.3
            },
            'agent_types': {
                'voa': {'priority': 1, 'max_tasks': 1},
                'mla': {'priority': 2, 'max_tasks': 2},
                'ca': {'priority': 3, 'max_tasks': 3},
                'fa': {'priority': 4, 'max_tasks': 2}
            }
        }

    def agent_status_callback(self, msg: AgentStatus) -> None:
        """Receive and process agent status updates."""
        agent_key = f"{msg.agent_id}"
        self.agents[agent_key] = AgentInfo(
            agent_id=msg.agent_id,
            agent_type=msg.agent_type,
            state=msg.state,
            health=msg.health,
            task_id=msg.task_id,
            capabilities=list(msg.capabilities),
            last_update=time.time()
        )
        
        # Log significant status changes
        if msg.state == 'error':
            self.get_logger().warn(f"⚠ Agent {msg.agent_id} entered ERROR state")
        elif msg.health < self.health_threshold:
            self.get_logger().warn(f"⚠ Agent {msg.agent_id} health critical: {msg.health:.2f}")

    def allocate_task_callback(self, request: AllocateTask.Request, 
                               response: AllocateTask.Response) -> AllocateTask.Response:
        """Handle task allocation requests from agents."""
        task_id = request.task_id
        target_agent = request.target_agent
        priority = request.priority

        # Validate target agent exists and is healthy
        if target_agent not in self.agents:
            response.success = False
            response.rejection_reason = f"Agent {target_agent} not found"
            self.get_logger().warn(response.rejection_reason)
            return response

        agent = self.agents[target_agent]
        
        # Check agent health
        if agent.health < self.health_threshold:
            response.success = False
            response.rejection_reason = f"Agent health too low: {agent.health:.2f}"
            return response

        # Check workload
        agent_tasks = [t for t, a in self.active_tasks.items() if a == target_agent]
        max_tasks = self.policies['agent_types'].get(agent.agent_type, {}).get('max_tasks', 1)
        
        if len(agent_tasks) >= max_tasks:
            response.success = False
            response.rejection_reason = f"Agent workload at capacity ({len(agent_tasks)}/{max_tasks})"
            return response

        # Allocate task
        self.active_tasks[task_id] = target_agent
        response.success = True
        response.allocation_id = f"{task_id}_{target_agent}"
        
        self.get_logger().info(f"✓ Task {task_id} allocated to {target_agent} (priority: {priority})")
        return response

    def query_agent_status_callback(self, request: QueryAgentStatus.Request,
                                   response: QueryAgentStatus.Response) -> QueryAgentStatus.Response:
        """Query agent status and capabilities."""
        agent_id = request.agent_id
        query_type = request.query_type

        if agent_id not in self.agents:
            response.response_status = "not_found"
            return response

        agent = self.agents[agent_id]
        response.response_status = "success"

        if query_type == 'capabilities':
            response.agent_info = agent.capabilities
        elif query_type == 'state':
            response.agent_info = [agent.state, str(agent.health)]
        elif query_type == 'utilization':
            num_tasks = len([t for t, a in self.active_tasks.items() if a == agent_id])
            response.agent_info = [str(num_tasks)]
        elif query_type == 'available_slots':
            max_tasks = self.policies['agent_types'].get(agent.agent_type, {}).get('max_tasks', 1)
            num_tasks = len([t for t, a in self.active_tasks.items() if a == agent_id])
            available = max(0, max_tasks - num_tasks)
            response.agent_info = [str(available)]

        return response

    def monitor_system_health(self) -> None:
        """Monitor overall system health and detect failures."""
        # Check for stale agent updates (no heartbeat for timeout period)
        current_time = time.time()
        failed_agents = []
        
        for agent_id, agent in list(self.agents.items()):
            if current_time - agent.last_update > self.centralized_timeout:
                failed_agents.append(agent_id)
                self.get_logger().warn(f"⚠ Agent {agent_id} heartbeat timeout")

        # Automatically enter degraded/emergency mode based on failures
        total_agents = len(self.agents)
        healthy_agents = total_agents - len(failed_agents)
        
        if total_agents > 0:
            health_ratio = healthy_agents / total_agents
            
            if health_ratio < self.policies['system_degradation']['emergency_threshold']:
                self.system_mode = SystemMode.EMERGENCY
            elif health_ratio < self.policies['system_degradation']['degraded_threshold']:
                self.system_mode = SystemMode.DEGRADED
            else:
                self.system_mode = SystemMode.NORMAL

    def check_centralized_communication(self) -> None:
        """Check if centralized communication is still viable."""
        current_time = time.time()
        
        # If too long since last external heartbeat, switch to local_only mode
        if self.system_mode == SystemMode.EMERGENCY and (current_time - self.last_heartbeat_time) > (self.centralized_timeout * 2):
            self.system_mode = SystemMode.LOCAL_ONLY
            self.get_logger().info("🔴 Switching to LOCAL_ONLY mode - centralized coordination lost")

    def publish_system_state(self) -> None:
        """Publish current system state for all agents."""
        msg = SystemState()
        msg.robot_id = self.robot_id
        msg.active_tasks = list(self.active_tasks.keys())
        msg.system_mode = self.system_mode.value
        msg.num_active_agents = len(self.agents)
        msg.failed_agents = [
            agent_id for agent_id, agent in self.agents.items()
            if agent.health < self.health_threshold
        ]
        msg.timestamp = self.get_clock().now().to_msg()

        # Calculate overall system health (average of all agent healths)
        if self.agents:
            msg.system_health = sum(a.health for a in self.agents.values()) / len(self.agents)
        else:
            msg.system_health = 0.0

        self.system_state_pub.publish(msg)

    def resolve_conflicts(self, task_id: str, agents: List[str]) -> Tuple[str, str]:
        """
        Resolve conflicts between agents using policy rules.
        Returns: (winning_agent, resolution_rule)
        """
        if not agents:
            return "", "no_conflict"

        # Apply priority-based conflict resolution
        if self.policies['conflict_resolution']['priority_override']:
            agent_priorities = []
            for agent_id in agents:
                if agent_id in self.agents:
                    agent_type = self.agents[agent_id].agent_type
                    priority = self.policies['agent_types'].get(agent_type, {}).get('priority', 999)
                    health = self.agents[agent_id].health
                    agent_priorities.append((agent_id, priority, health))
                else:
                    agent_priorities.append((agent_id, 999, 0.0))

            # Sort by priority (lower is better), then by health (higher is better)
            agent_priorities.sort(key=lambda x: (x[1], -x[2]))
            winning_agent = agent_priorities[0][0]
            
            # Publish conflict resolution
            conf_msg = ConflictResolution()
            conf_msg.robot_id = self.robot_id
            conf_msg.conflicting_agents = agents
            conf_msg.conflicting_tasks = [task_id]
            conf_msg.resolution_rule = "priority_override"
            conf_msg.recommended_action = f"Agent {winning_agent} takes priority"
            self.conflict_pub.publish(conf_msg)
            
            return winning_agent, "priority_override"

        return agents[0], "first_come_first_served"


def main(args=None):
    rclpy.init(args=args)
    node = SupervisoryControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Supervisory Control Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
