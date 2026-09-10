# Одно направленное ребро графа дорог
string   name              # "AB", "BD", ... — направление от A к B
bool     is_visited
string[] forbidden_entry   # список имён рёбер, с которых нельзя въехать на это
bool     have_parking      # знак «Остановка» на этом ребре
bool     have_passengers   # знак «Посадка пассажиров» на этом ребре
