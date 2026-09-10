from typing import Dict, List
from city_interfaces.msg import RoadInfo


# 16 направленных рёбер симметричного графа 5 перекрёстков.
# A, B, C, D — угловые T-образные, E — центральный крестовой.
# Порядок важен: по нему строится Graph.msg и стабильно индексируются рёбра.
EDGE_NAMES: List[str] = [
    'AB', 'BA', 'BC', 'CB', 'CD', 'DC', 'DA', 'AD',
    'AE', 'EA', 'BE', 'EB', 'CE', 'EC', 'DE', 'ED',
]

# Атрибуты, которые мы вообще умеем менять через /graph_updates
BOOL_ATTRS = {'is_visited', 'have_parking', 'have_passengers'}
LIST_ATTRS = {'forbidden_entry'}

_TRUE_TOKENS  = {'yes', 'true', '1', 'on'}
_FALSE_TOKENS = {'no',  'false', '0', 'off'}


class Road:
    """Одно направленное ребро графа."""

    __slots__ = ('name', 'is_visited', 'forbidden_entry',
                 'have_parking', 'have_passengers')

    def __init__(self, name: str) -> None:
        self.name = name
        self.is_visited = False
        self.forbidden_entry: List[str] = []
        self.have_parking = False
        self.have_passengers = False

    def to_msg(self) -> RoadInfo:
        m = RoadInfo()
        m.name = self.name
        m.is_visited = self.is_visited
        m.forbidden_entry = list(self.forbidden_entry)
        m.have_parking = self.have_parking
        m.have_passengers = self.have_passengers
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
    return {name: Road(name) for name in EDGE_NAMES}
