import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from city_interfaces.srv import GetNextGoal
from city_interfaces.action import MoveCommand

from city_graph.graph_data import DEFAULT_GEOMETRY


# Стартовое ребро: робот стоит на уголке AB, смотрит на B.
# Первый маневр — не по формуле, а жёстко GO_STRAIGHT (a - b).
START_EDGE_NAME = 'AB'


def normalize_deg(angle: float) -> float:
    """Приводит угол к диапазону [0, 360)."""
    return angle % 360.0


def effective_heading(orientation: float, shape: str) -> float:
    """
    Курс робота в момент окончания ребра с данной .orientation и .shape.
    Для straight — это .orientation;
    для curved_right — .orientation - 90°;
    для curved_left  — .orientation + 90°.
    """
    if shape == 'curved_right':
        return normalize_deg(orientation - 90.0)
    if shape == 'curved_left':
        return normalize_deg(orientation + 90.0)
    return normalize_deg(orientation)


def angle_delta(target_deg: float, base_deg: float) -> int:
    """
    Разница (target - base) в градусах, нормализованная в [0, 360),
    округлённая до кратного 90°. Для неточных значений вроде 89.999
    даст ровно 90.
    """
    raw = normalize_deg(target_deg - base_deg)
    return int(round(raw / 90.0)) * 90 % 360


class GoalFragmentatorNode(Node):

    def __init__(self) -> None:
        super().__init__('goal_fragmentator_node')

        # --- параметры геометрии полигона ---
        # a — расстояние между центрами соседних перекрёстков, мм
        # b — ширина полосы, мм
        self.declare_parameter('intercross_distance_mm', 1600)
        self.declare_parameter('lane_width_mm', 400)
        self._a = int(self.get_parameter('intercross_distance_mm').value)
        self._b = int(self.get_parameter('lane_width_mm').value)

        # --- клиенты ---
        self._goal_client = self.create_client(GetNextGoal, '/get_next_goal')
        self._move_client = ActionClient(self, MoveCommand, '/move_command')

        # --- состояние ---
        start_orient, start_shape = DEFAULT_GEOMETRY[START_EDGE_NAME]
        self._prev_edge: tuple = (START_EDGE_NAME, start_orient, start_shape)
        # Стартовый маневр: от уголка AB до ближнего правого квадратика B
        self._queue: list = [('GO_STRAIGHT', '', self._a - self._b)]
        self._mission_complete = False
        self._waiting_result = False

        self.get_logger().info(
            f'goal_fragmentator_node up: a={self._a}mm, b={self._b}mm, '
            f'start_prev={self._prev_edge}, '
            f'initial_queue={self._queue}'
        )

        # Ждём появления action-сервера STM, затем запускаем цепочку.
        self.get_logger().info('waiting for /move_command action server...')
        self._move_client.wait_for_server()
        self.get_logger().info('action server available — kicking off')
        self._execute_next()

    # ------------------------------------------------------------------ main loop
    def _execute_next(self) -> None:
        """Выталкивает следующую команду из очереди либо запрашивает новый план."""
        if self._waiting_result or self._mission_complete:
            return

        if not self._queue:
            # Все низкоуровневые команды выполнены, робот стоит на
            # новом перекрёстке. Спрашиваем у goal_manager следующую цель.
            self._fetch_and_plan()
            return

        command = self._queue.pop(0)
        self._send_to_stm(command)

    # ------------------------------------------------------------------ goal request
    def _fetch_and_plan(self) -> None:
        if not self._goal_client.service_is_ready():
            self.get_logger().warn(
                '/get_next_goal not ready; retrying in 0.5 s'
            )
            self.create_timer(0.5, self._retry_fetch)
            return

        req = GetNextGoal.Request()
        future = self._goal_client.call_async(req)
        future.add_done_callback(self._on_goal_response)

    def _retry_fetch(self) -> None:
        # Одноразовый таймер-обёртка. Пытаемся снова и, если сервис готов,
        # уходим в обычную ветку.
        if self._goal_client.service_is_ready():
            self._fetch_and_plan()

    def _on_goal_response(self, future) -> None:
        resp = future.result()
        if resp is None:
            self.get_logger().error('get_next_goal returned None')
            self._mission_complete = True
            return
        if not resp.has_goal:
            self.get_logger().info('goal_manager: no more goals — mission complete')
            self._mission_complete = True
            return

        target = (resp.edge_name, resp.orientation, resp.shape)
        new_commands = self._compute_maneuver(self._prev_edge, target)
        if not new_commands:
            self.get_logger().error(
                f'empty maneuver for target {target} — aborting'
            )
            self._mission_complete = True
            return

        self._queue.extend(new_commands)
        self._prev_edge = target
        self.get_logger().info(
            f'target={target[0]} (orient {target[1]}°, {target[2]}) → '
            f'maneuver={new_commands}'
        )
        self._execute_next()

    # ------------------------------------------------------------------ maneuver
    def _compute_maneuver(self, prev_edge, target_edge) -> list:
        prev_name, prev_orient, prev_shape = prev_edge
        tgt_name, tgt_orient, tgt_shape = target_edge

        prev_eff = effective_heading(prev_orient, prev_shape)
        d = angle_delta(tgt_orient, prev_eff)

        self.get_logger().debug(
            f'compute: prev={prev_name}({prev_orient}°,{prev_shape},'
            f'eff={prev_eff}°) target={tgt_name}({tgt_orient}°,{tgt_shape}) '
            f'Δ={d}°'
        )

        if d == 0:
            return [('GO_STRAIGHT', '', self._a)]
        if d == 270:    # поворот направо на 90°
            return [
                ('TURN', 'RIGHT', 0),
                ('GO_STRAIGHT', '', self._a - self._b),
            ]
        if d == 90:     # поворот налево на 90°
            return [
                ('GO_STRAIGHT', '', self._b),
                ('TURN', 'LEFT', 0),
                ('GO_STRAIGHT', '', self._a),
            ]
        if d == 180:    # разворот
            return [
                ('TURN', 'LEFT', 0),
                ('GO_STRAIGHT', '', self._b),
                ('TURN', 'LEFT', 0),
                ('GO_STRAIGHT', '', self._a - self._b),
            ]

        self.get_logger().error(
            f'unexpected Δ={d}° between {prev_name} and {tgt_name}'
        )
        return []

    # ------------------------------------------------------------------ STM
    def _send_to_stm(self, command) -> None:
        command_type, direction, distance = command

        goal = MoveCommand.Goal()
        goal.command_type = command_type
        goal.direction = direction
        goal.distance_mm = int(distance)

        self.get_logger().info(
            f'→ STM: {command_type} '
            f'{direction + " " if direction else ""}{distance}'
        )
        self._waiting_result = True
        send_future = self._move_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('STM rejected goal')
            self._waiting_result = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        # Фидбэк не обрабатываем. Просто продолжаем цепочку.
        self._waiting_result = False
        self._execute_next()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalFragmentatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
