# Положение робота в графе дорог

string  position_type   # "node" | "edge" | "unknown"
string  node_name       # заполнено, если position_type=="node"
string  edge_name       # заполнено, если position_type=="edge"
string  entry_edge      # ребро, с которого робот приехал в node_name;
                        # пусто, если position_type=="edge" или на старте
float32 progress        # 0.0..1.0 — доля пройденного по ребру
int32   heading_deg     # текущий курс, CCW от стартового взгляда
