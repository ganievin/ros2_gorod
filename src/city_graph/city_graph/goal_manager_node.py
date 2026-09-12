import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from city_interfaces.msg import Graph, RoadInfo, TrackingInfo
from city_interfaces.srv import GetNextGoal

from city_graph.graph_data import Road
from city_graph.planner import bfs_find_target


# ---------------------------------------------------------------------------
# QoS: подписываемся на /graph, который graph_storage публикует с
# TRANSIENT_LOCAL — тогда мы сразу получаем самую свежую версию графа при
# старте, не дожидаясь первой правки.
# ---------------------------------------------------------------------------
GRAPH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

TRACKING_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class GoalManagerNode(Node):

    def __init__(self) -> None:
        super().__init__('goal_manager_node')

        # --- состояние ---
        self._roads: dict = {}            # name -> Road, полностью из /graph
        self._graph_received: bool = False
        self._current: TrackingInfo = TrackingInfo()
        self._current.position_type = 'unknown'
        self._buffer: list = []           # FIFO из имён рёбер

        # --- ROS-обвязка ---
        self._sub_graph = self.create_subscription(
            Graph, '/graph', self._on_graph, GRAPH_QOS
        )
        self._sub_tracking = self.create_subscription(
            TrackingInfo, '/tracking_topic', self._on_tracking, TRACKING_QOS
        )
        self._srv = self.create_service(
            GetNextGoal, '/get_next_goal', self._on_get_next_goal
        )

        self.get_logger().info('goal_manager_node up')

    # ------------------------------------------------------------------ subs
    def _on_graph(self, msg: Graph) -> None:
        self._roads = {r.name: self._to_road(r) for r in msg.roads}
        self._graph_received = True
        self.get_logger().debug(
            f'graph received: {len(self._roads)} edges, '
            f'buffer={self._buffer}'
        )

    def _on_tracking(self, msg: TrackingInfo) -> None:
        self._current = msg

    # ------------------------------------------------------------------ srv
    def _on_get_next_goal(self, request, response):
        # 1. Гарантируем наличие графа
        if not self._graph_received:
            self.get_logger().warn('request before first /graph — no goal')
            response.has_goal = False
            return response

        # 2. Если буфер пуст — пополняем его через планировщик
        if not self._buffer:
            self._plan_new_route()

        # 3. Миссия завершена?
        if not self._buffer:
            self.get_logger().info('buffer empty after planning → mission complete')
            response.has_goal = False
            return response

        # 4. Выдаём следующую цель из буфера (FIFO)
        edge_name = self._buffer.pop(0)
        edge = self._roads.get(edge_name)
        if edge is None:
            # К этому моменту граф мог обновиться так, что ребро исчезло.
            # В нашей топологии такого не бывает, но подстрахуемся.
            self.get_logger().warn(f'edge {edge_name} vanished — skip')
            response.has_goal = False
            return response

        response.has_goal = True
        response.edge_name = edge.name
        response.orientation = edge.orientation
        response.shape = edge.shape

        self.get_logger().info(
            f'goal: {edge.name} ({edge.orientation}°, {edge.shape}); '
            f'remaining in buffer: {self._buffer}'
        )
        return response

    # ------------------------------------------------------------------ core
    def _plan_new_route(self) -> None:
        """
        Наполняет self._buffer новым путём. Если путь не найден или миссия
        завершена — оставляет буфер пустым (это сигнал "нет целей").
        """
        # ---- 0. Миссия завершена? Мы на парковочном ребре. ----
        if self._current.position_type == 'edge':
            cur_edge = self._roads.get(self._current.edge_name)
            if cur_edge is not None and cur_edge.have_parking:
                self.get_logger().info(
                    f'already on parking edge {cur_edge.name} → mission complete'
                )
                return

        # ---- 1. Определяем фазу миссии ----
        passenger_count = sum(
            1 for r in self._roads.values() if r.have_passengers
        )
        parking_known = any(r.have_parking for r in self._roads.values())

        if passenger_count >= 2 and parking_known:
            # Фаза В: все пассажиры собраны, едем на парковку
            target_predicate = lambda r: r.have_parking
            phase = 'PARKING'
        elif passenger_count >= 2 and not parking_known:
            # Парковка ещё не найдена — исследуем дальше
            target_predicate = lambda r: True
            phase = 'EXPLORE_FOR_PARKING'
        else:
            # Фаза А/Б: пассажиров ещё не хватает — исследуем
            target_predicate = lambda r: True
            phase = 'EXPLORE_FOR_PASSENGERS'

        # ---- 2. Определяем стартовый узел ----
        start_node = self._resolve_start_node()
        if start_node is None:
            self.get_logger().warn(
                f'cannot plan: current position={self._current.position_type}, '
                f'node="{self._current.node_name}", edge="{self._current.edge_name}"'
            )
            return

        self.get_logger().info(
            f'planning from node {start_node}, phase={phase}, '
            f'passengers={passenger_count}, parking_known={parking_known}'
        )

        # ---- 3. BFS ----
        path = bfs_find_target(start_node, self._roads, target_predicate)
        if path is None:
            self.get_logger().warn('BFS found no route — buffer stays empty')
            return

        self._buffer.extend(path)
        self.get_logger().info(f'planned path: {path}')

    # ------------------------------------------------------------------ helpers
    def _resolve_start_node(self):
        pos = self._current
        if pos.position_type == 'node':
            return pos.node_name or None
        if pos.position_type == 'edge':
            # Мы на ребре — считаем, что движемся к его концу (вторая буква)
            if pos.edge_name and len(pos.edge_name) == 2:
                return pos.edge_name[1]
        return None

    @staticmethod
    def _to_road(msg: RoadInfo) -> Road:
        r = Road(msg.name, msg.orientation, msg.shape)
        r.is_visited = msg.is_visited
        r.forbidden_entry = list(msg.forbidden_entry)
        r.have_parking = msg.have_parking
        r.have_passengers = msg.have_passengers
        return r


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
