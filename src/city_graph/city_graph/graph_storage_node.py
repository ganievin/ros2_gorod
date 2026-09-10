import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from city_interfaces.msg import Graph, GraphUpdate
from city_interfaces.srv import GetGraph

from city_graph.graph_data import (
    EDGE_NAMES, BOOL_ATTRS, LIST_ATTRS,
    Road, build_default_graph, parse_bool,
)


class GraphStorageNode(Node):
    """
    Хранит граф дорог полигона и предоставляет доступ к нему.

    Вход:   /graph_updates (GraphUpdate) — атомарные изменения атрибутов рёбер
    Выход:  /get_graph     (GetGraph)    — по запросу отдаёт полный граф
    """

    def __init__(self) -> None:
        super().__init__('graph_storage_node')

        # --- состояние: единственная точка правды ---
        # Все изменения графа происходят в колбэке _on_update,
        # колбэк _on_get_graph только читает. В rclpy при single-threaded
        # executor'е этого достаточно, чтобы не блокироваться.
        self._roads = build_default_graph()

        # --- QoS: гарантируем доставку каждого обновления ---
        update_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=64,
        )

        self._sub = self.create_subscription(
            GraphUpdate,
            '/graph_updates',
            self._on_update,
            update_qos,
        )

        self._srv = self.create_service(
            GetGraph,
            '/get_graph',
            self._on_get_graph,
        )

        self.get_logger().info(
            f'graph_storage_node up: {len(self._roads)} directed edges'
        )

    # ------------------------------------------------------------------ sub
    def _on_update(self, msg: GraphUpdate) -> None:
        road = self._roads.get(msg.road_name)
        if road is None:
            self.get_logger().warn(
                f'ignoring update: unknown road "{msg.road_name}"'
            )
            return

        attr = msg.attribute_name

        try:
            if attr in BOOL_ATTRS:
                self._apply_bool(road, attr, msg.values)
            elif attr in LIST_ATTRS:
                self._apply_list(road, attr, msg.values)
            else:
                self.get_logger().warn(
                    f'ignoring update: unknown attribute "{attr}"'
                )
                return
        except ValueError as exc:
            self.get_logger().warn(
                f'rejecting update for {road.name}.{attr}: {exc}'
            )
            return

        self.get_logger().info(
            f'update: {road.name}.{attr} <- {list(msg.values)}'
        )

    def _apply_bool(self, road: Road, attr: str, values) -> None:
        setattr(road, attr, parse_bool(values))

    def _apply_list(self, road: Road, attr: str, values) -> None:
        # Семантика: массив values = «полный список». Пустой массив — очистить.
        # Это позволяет sign_handler'у перезаписывать forbidden_entry целиком,
        # если правило знака поменялось.
        validated = []
        for v in values:
            if v not in self._roads:
                # не роняем ноду из-за одной плохой ссылки
                self.get_logger().warn(
                    f'{road.name}.{attr}: unknown referenced edge "{v}"'
                )
                continue
            if v == road.name:
                self.get_logger().warn(
                    f'{road.name}.{attr}: self-reference ignored'
                )
                continue
            validated.append(v)
        setattr(road, attr, validated)

    # ------------------------------------------------------------------ srv
    def _on_get_graph(self, request, response):
        graph = Graph()
        # Порядок строго по EDGE_NAMES — goal_manager может на него опираться
        graph.roads = [self._roads[name].to_msg() for name in EDGE_NAMES]
        response.graph = graph
        return response

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraphStorageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
