#!/bin/bash
# Окружение для воркспейса gorod-clean (ROS2 Jazzy)

# 1. ROS2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✅ ROS2 Jazzy активирован"
else
    echo "❌ ROS2 Jazzy не найден в /opt/ros/jazzy/"
    return 1 2>/dev/null || exit 1
fi

# 2. Воркспейс gorod-clean
WORKSPACE="$HOME/cvat/gorod-clean"
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
    echo "✅ Воркспейс gorod-clean активирован"
else
    echo "⚠️  $WORKSPACE/install/setup.bash не найден"
    echo "    Сначала соберите воркспейс: cd $WORKSPACE && colcon build"
fi

# 3. Проверка
echo ""
echo "Python: $(which python3) — $(python3 --version 2>&1)"
echo "ROS2: $(command -v ros2 || echo 'не найден')"
echo ""
echo "Доступные пакеты воркспейса:"
ros2 pkg list 2>/dev/null | grep -E "city_graph|city_interfaces|trafic_detection" || echo "  (пока ничего — соберите colcon build)"

