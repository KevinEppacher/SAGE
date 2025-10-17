from concurrent.futures import ThreadPoolExecutor
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import time

# ANSI color codes
BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"

class LifecycleController(Node):
    def __init__(self):
        super().__init__('lifecycle_controller')

        self.declare_parameter('target_nodes', ['/target_nodes'])
        self.declare_parameter('timeout_sec', 90.0)
        self.target_nodes = self.get_parameter('target_nodes').get_parameter_value().string_array_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        self.log_info(f'Starting lifecycle controller for {len(self.target_nodes)} node(s)...')
        self.managers = [LifecycleNodeManager(self, name, self.timeout_sec) for name in self.target_nodes]

        self.run_all()

    def run_all(self):
        for mgr in self.managers:
            mgr.run()

    def log_info(self, msg): self.get_logger().info(f'{BLUE}[LifecycleController] {msg}{RESET}')
    def log_warn(self, msg): self.get_logger().warn(f'{YELLOW}[LifecycleController] {msg}{RESET}')
    def log_error(self, msg): self.get_logger().error(f'{RED}[LifecycleController] {msg}{RESET}')


class LifecycleNodeManager:
    def __init__(self, node, target_node, timeout_sec):
        self.node = node
        self.target_node = target_node
        self.timeout_sec = timeout_sec
        self.cli_change = node.create_client(ChangeState, f'{target_node}/change_state')
        self.cli_get = node.create_client(GetState, f'{target_node}/get_state')

    def log(self, level, msg, color):
        prefix = f'{BOLD}[{self.target_node}]{RESET}'
        getattr(self.node.get_logger(), level)(f'{color}{prefix} {msg}{RESET}')

    def run(self):
        self.log('info', 'Connecting to lifecycle services...', YELLOW)
        start_time = time.time()
        while time.time() - start_time < self.timeout_sec:
            if self.cli_change.service_is_ready() and self.cli_get.service_is_ready():
                self.log('info', 'Services available', GREEN)
                break
            time.sleep(0.5)
        else:
            self.log('error', 'Timeout while waiting for services', RED)
            return

        self.transition(Transition.TRANSITION_CONFIGURE, 'CONFIGURE')
        self.transition(Transition.TRANSITION_ACTIVATE, 'ACTIVATE')

    def transition(self, transition_id, label):
        self.log('info', f'→ Transition: {label}...', BLUE)
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.cli_change.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout_sec)

        if future.result() and future.result().success:
            state = self.get_state()
            self.log('info', f'✔ Successfully transitioned → {state}', GREEN)
        else:
            self.log('error', f'✘ Failed to transition to {label}', RED)

    def get_state(self):
        req = GetState.Request()
        future = self.cli_get.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout_sec)
        return future.result().current_state.label if future.result() else 'unknown'


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
