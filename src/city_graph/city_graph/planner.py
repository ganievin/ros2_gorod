from collections import deque
from typing import Callable, Dict, List, Optional

from city_graph.graph_data import Road


MAX_DEPTH = 16  # длина пути не может превышать число рёбер


def outgoing_edges(node: str, roads: Dict[str, Road]) -> List[str]:
    """Все рёбра, выходящие из узла node, в алфавитном порядке."""
    return sorted(name for name in roads if name[0] == node)


def bfs_find_target(
    start_node: str,
    roads: Dict[str, Road],
    target_predicate: Callable[[Road], bool],
    max_depth: int = MAX_DEPTH,
) -> Optional[List[str]]:
    """
    BFS от start_node. Возвращает кратчайший (в порядке BFS с лексикографическим
    tie-break) путь [e1, e2, ..., ek], у которого ПОСЛЕДНЕЕ ребро ek:
      * не is_visited
      * удовлетворяет target_predicate(ek)
    Промежуточные рёбра могут быть посещёнными (транзит).
    Учитывается forbidden_entry: цепочка недопустима, если предыдущее
    ребро числится в forbidden_entry следующего.
    """
    visited_nodes: set = set()
    queue: deque = deque()
    queue.append((start_node, []))

    while queue:
        node, path = queue.popleft()
        if len(path) >= max_depth:
            continue

        for edge_name in outgoing_edges(node, roads):
            edge = roads[edge_name]

            # Проверка ПДД: с предыдущего ребра нельзя въехать на это
            if path and path[-1] in edge.forbidden_entry:
                continue

            new_path = path + [edge_name]

            # Проверка «это наша цель?»
            if not edge.is_visited and target_predicate(edge):
                return new_path

            # Иначе — идём дальше
            next_node = edge_name[1]  # вторая буква имени ребра = узел-назначение
            if next_node in visited_nodes:
                continue
            visited_nodes.add(next_node)
            queue.append((next_node, new_path))

    return None
