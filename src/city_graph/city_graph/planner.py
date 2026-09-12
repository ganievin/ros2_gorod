from collections import deque
from typing import Callable, Dict, List, Optional

from city_graph.graph_data import Road


MAX_DEPTH = 16  # длина пути не может превышать число рёбер в графе


def outgoing_edges(node: str, roads: Dict[str, Road]) -> List[str]:
    """Все рёбра, выходящие из узла node, в алфавитном порядке."""
    return sorted(name for name in roads if name[0] == node)


def bfs_find_target(
    start_node: str,
    roads: Dict[str, Road],
    target_predicate: Callable[[Road], bool],
    entry_edge: Optional[str] = None,
    require_unvisited: bool = True,
    max_depth: int = MAX_DEPTH,
) -> Optional[List[str]]:
    """
    BFS от start_node. Возвращает кратчайший (в порядке BFS с
    лексикографическим tie-break) путь [e1, e2, ..., ek], у которого
    ПОСЛЕДНЕЕ ребро ek удовлетворяет target_predicate(ek).

    require_unvisited:
      * True  — целевое ребро обязано иметь is_visited == False.
                Используется в фазах исследования (EXPLORE_*): ищем
                рёбра, где робот ещё не был.
      * False — целевым может быть любое ребро, включая уже пройденное.
                Используется в фазе PARKING: конечная цель — вернуться
                на ребро со знаком парковки, даже если оно уже
                посещалось раньше.

    Промежуточные рёбра пути могут быть посещёнными (транзит).

    entry_edge — имя ребра, с которого робот приехал в start_node.
    Если оно задано, то forbidden_entry применяется и на первом шаге.
    Иначе первый шаг проходит без проверки (нет информации о том, откуда
    робот въехал на узел).

    Учитывается forbidden_entry: цепочка недопустима, если предыдущее
    ребро числится в forbidden_entry следующего.
    """
    visited_nodes: set = set()
    queue: deque = deque()
    # Очередь несёт тройку (node, path, prev_edge):
    #   node      — текущий узел;
    #   path      — рёбра, которыми сюда пришли;
    #   prev_edge — ребро, которым завершается path (или entry_edge
    #               на старте).
    queue.append((start_node, [], entry_edge))

    while queue:
        node, path, prev_edge = queue.popleft()
        if len(path) >= max_depth:
            continue

        for edge_name in outgoing_edges(node, roads):
            edge = roads[edge_name]

            # Проверка ПДД: с предыдущего ребра нельзя въехать на это.
            # Работает и на первом шаге, если задан entry_edge.
            if prev_edge is not None and prev_edge in edge.forbidden_entry:
                continue

            new_path = path + [edge_name]

            edge_ok = (not require_unvisited) or (not edge.is_visited)
            if edge_ok and target_predicate(edge):
                return new_path

            next_node = edge_name[1]  # вторая буква = узел-назначение
            if next_node in visited_nodes:
                continue
            visited_nodes.add(next_node)
            queue.append((next_node, new_path, edge_name))

    return None
