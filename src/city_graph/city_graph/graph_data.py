from typing import Dict, List

from city_interfaces.msg import RoadInfo, Graph


# 16 направленных рёбер. Порядок фиксирован — по нему строится Graph.msg
# и стабильно индексируются рёбра у подписчиков.
EDGE_NAMES: List[str] = [
    'AB', 'BA', 'BC', 'CB', 'CD', 'DC', 'DA', 'AD',
    'AE', 'EA', 'BE', 'EB', 'CE', 'EC', 'DE', 'ED',
]

# (orientation_deg, shape) для каждого ребра.
#
# orientation — направление первой половины ребра (у L-образных) или
# направления движения вдоль ребра (у прямых), в градусах CCW от
# стартового взгляда робота.
#
# shape принимает значения:
#   "straight"      — прямой отрезок;
#   "curved_right"  — L-образная, при движении поворот направо (CW);
#   "curved_left"   — L-образная, при движении поворот налево (CCW).
#
# Стартовые условия: робот стоит на уголке AB, смотрит на B, курс = 0°.
# Из того, что AB.orientation = 90° и что робот пришёл из A по курсу 90°,
# следует, что AB поворачивает направо → "curved_right". Обратные к
# периметру рёбра поворачивают в противоположную сторону → "curved_left".
# Прямые рёбра — "straight".
DEFAULT_GEOMETRY: Dict[str, tuple] = {
    'AB': (90,  'curved_right'),
    'BA': (180, 'curved_left'),
    'BC': (0,   'curved_right'),
    'CB': (90,  'curved_left'),
    'CD': (270, 'curved_right'),
    'DC': (0,   'curved_left'),
    'DA': (180, 'curved_right'),
    'AD': (270, 'curved_left'),
    'AE': (0,   'straight'),
    'EA': (180, 'straight'),
    'BE': (270, 'straight'),
    'EB': (90,  'straight'),
    'CE': (180, 'straight'),
    'EC': (0,   'straight'),
    'DE': (90,  'straight'),
    'ED': (270, 'straight'),
}

BOOL_ATTRS = {'is_visited', 'have_parking', 'have_passengers'}
LIST_ATTRS = {'forbidden_entry'}

_TRUE_TOKENS  = {'yes', 'true', '1', 'on'}
_FALSE_TOKENS = {'no',  'false', '0', 'off'}


class Road:
    """Одно направленное ребро графа."""

    __slots__ = ('name', 'is_visited', 'forbidden_entry',
                 'have_parking', 'have_passengers',
                 'orientation', 'shape')

    def __init__(self, name: str, orientation: int, shape: str) -> None:
        self.name = name
        self.is_visited = False
        self.forbidden_entry: List[str] = []
        self.have_parking = False
        self.have_passengers = False
        self.orientation = int(orientation)
        self.shape = shape

    def to_msg(self) -> RoadInfo:
        m = RoadInfo()
        m.name = self.name
        m.is_visited = self.is_visited
        m.forbidden_entry = list(self.forbidden_entry)
        m.have_parking = self.have_parking
        m.have_passengers = self.have_passengers
        m.orientation = self.orientation
        m.shape = self.shape
        return m


def parse_bool(values) -> bool:
    if not values:
        raise ValueError('empty values for boolean attribute')
    v = values[0].strip().lower()
    if v in _TRUE_TOKENS:
        return True
    if v in _FALSE_TOKENS:
        return False
    raise ValueError(f'cannot parse "{v}" as bool')


def build_default_graph() -> Dict[str, Road]:
    roads: Dict[str, Road] = {}
    for name in EDGE_NAMES:
        orient, shape = DEFAULT_GEOMETRY[name]
        roads[name] = Road(name, orient, shape)
    return roads


def build_graph_msg(roads: Dict[str, Road]) -> Graph:
    msg = Graph()
    msg.roads = [roads[name].to_msg() for name in EDGE_NAMES]
    return msg
