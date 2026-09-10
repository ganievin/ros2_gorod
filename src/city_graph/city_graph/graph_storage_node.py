import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from city_interfaces.msg import Graph, GraphUpdate

from city_graph.graph_data import (
    BOOL_ATTRS, LIST_ATTRS, Road,
    build_default_graph, build_graph_msg, parse_bool,
)


class GraphStorageNode(Node):
    """
    Хранит граф дорог и публикует его актуальную версию.

    Вход:   /graph_updates (GraphUpdate) — атомарные правки атрибутов рёбер
    Выход:  /graph         (Graph)       — полный граф после каждой правки
                                            и один раз при старте.

    Оба топика — RELIABLE. /graph дополнительно TRANSIENT_LOCAL(depth=1):
    поздний подписчик (например, только что запустившийся goal_manager_node)
    сразу получает последнюю опубликованную версию графа.
    """

    def __init__(self) -> None:
        super().__init__('graph_storage_node')

        self._roads = build_default_graph()

        update_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=64,
        )
        graph_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub = self.create_publisher(Graph, '/graph', graph_qos)
        self._sub = self.create_subscription(
            GraphUpdate, '/graph_updates', self._on_update, update_qos
        )

        # Первичная публикация — чтобы поздние подписчики не ждали первой правки.
        self._publish_graph()

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
                setattr(road, attr, parse_bool(msg.values))
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
        self._publish_graph()

    def _apply_list(self, road: Road, attr: str, values) -> None:
        # Полная замена списка. Пустой массив = очистить.
        validated = []
        for v in values:
            if v not in self._roads:
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

    # ------------------------------------------------------------------ pub
    def _publish_graph(self) -> None:
        self._pub.publish(build_graph_msg(self._roads))


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
