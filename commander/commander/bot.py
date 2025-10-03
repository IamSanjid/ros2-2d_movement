from typing import Callable
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import GetParameters
from rclpy.parameter_event_handler import ParameterEventHandler, ParameterEvent
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from interfaces.msg import CameraView, Point
from interfaces.srv import GetPos
from interfaces.action import Move

CAMERA_NODE = "Camera"
CAMERA_PARAMS = {
    # 'distance',
    # 'angle_x',
    # 'angle_y',
    # 'rotation',
    'view_step',
}

class Viewer(Node):

    def __init__(self):
        super().__init__('Viewer')

        self.props_update_callback: None | Callable[[Any], None] = None
        self.camera_view_bumps_callback: None | Callable[[CameraView], None] = None
        self._move_future = None

        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.bumps_subscription = self.create_subscription(
            CameraView,
            'ViewBumps',
            self.on_view_bumps,
            qos)
        self.pos_subscription = self.create_subscription(Point, 'PosUpdate', self.on_pos_update, qos)
        self.move_client = ActionClient(self, Move, 'Move')

        self.param_handler = ParameterEventHandler(self, qos)

        self.param_callback = self.param_handler.add_parameter_event_callback(callback=self.on_camera_change)

        self.props = {}
        self.camera_get_param = self.create_client(GetParameters, f'/{CAMERA_NODE}/get_parameters')
        self.camera_get_param.wait_for_service()
        self.camera_set_param = self.create_client(SetParameters, f'/{CAMERA_NODE}/set_parameters')
        self.camera_set_param.wait_for_service()
        self.body_get_pos = self.create_client(GetPos, 'GetPos')
        self.sync_with_camera_and_body()

        self.bumps_subscription

    def sync_with_camera_and_body(self):
        req = GetParameters.Request()
        req.names = CAMERA_PARAMS
        future = self.camera_get_param.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return
        resp: GetParameters.Response = future.result()
        for prop, value in zip(CAMERA_PARAMS, resp.values):
            self.props[prop] = parameter_value_to_python(value)

        req = GetPos.Request()
        future = self.body_get_pos.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        resp: GetPos.Response = future.result()
        self.props['pos_x'] = resp.pos.x
        self.props['pos_y'] = resp.pos.y
        self.get_logger().info("Bot props: {}".format(self.props))

    def move_bot(self, direction: int, step: int,
                 feedback_callback: Callable[[Move.Feedback], None] | None = None):
        if not self._move_future is None: return
        goal_msg = Move.Goal()
        goal_msg.from_pos = Point()
        goal_msg.from_pos.x = self.props['pos_x']
        goal_msg.from_pos.y = self.props['pos_y']
        goal_msg.direction = direction
        goal_msg.step = step

        self.move_client.wait_for_server()

        self._move_future = self.move_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        self._move_future.add_done_callback(self._move_goal_response_callback)

    def cancel_moving(self):
        if self._move_future is None: return
        if self._move_goal_handle is None: return
        
        self._move_goal_handle.cancel_goal_async()

        self._move_future = None
        self._move_goal_handle = None

    def _move_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Move rejected!')
            return

        self.get_logger().info('Bot is moving..')

        if self.props_update_callback is None:
            self._move_future = None
            return

        self._move_goal_handle = goal_handle
        # _move_future will now contain the future which should return on action finish
        self._move_future = self._move_goal_handle.get_result_async()
        self._move_future.add_done_callback(self._move_goal_reached)

    def _move_goal_reached(self, future):
        self._move_future = None
        self._move_goal_handle = None

        pos = future.result().result.new_pos
        self.props['pos_x'] = pos.x
        self.props['pos_y'] = pos.y

        self.props_update_callback(self.props)

    def on_view_bumps(self, msg: CameraView):
        if self.camera_view_bumps_callback is None: return
        self.camera_view_bumps_callback(msg)

    def on_pos_update(self, pos: Point):
        if self.props_update_callback is None: return
        self.props['pos_x'] = pos.x
        self.props['pos_y'] = pos.y
        self.props_update_callback(self.props)

    def on_camera_change(self, param_evt: ParameterEvent):
        if self.props_update_callback is None: return

        if param_evt.node != f"{CAMERA_NODE}" and param_evt.node != f"/{CAMERA_NODE}":
            return
        self.get_logger().info("Param changed: {}".format(param_evt.changed_parameters))

        for param in param_evt.changed_parameters:
            if not param.name in self.props: continue
            self.props[param.name] = parameter_value_to_python(param.value)

        self.props_update_callback(self.props)

    def destroy_node(self):
        try:
            self.param_handler.remove_parameter_event_callback(self.param_callback)
        except Exception:
            pass

        super().destroy_node()
