#!/bin/bash
# Активация ROS2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✅ ROS2 Jazzy активирован"
    # Проверка что ros2 доступен
    if command -v ros2 &> /dev/null; then
        ros2 --version
    else
        echo "⚠️  ros2 команда не найдена, но установка есть"
        echo "Попробуйте: source /opt/ros/jazzy/setup.bash"
    fi
else
    echo "❌ ROS2 Jazzy не найден в /opt/ros/jazzy/"
fi

# Активация виртуального окружения
if [ -f ~/cvat/plavki/ros_venv/bin/activate ]; then
    source ~/cvat/plavki/ros_venv/bin/activate
    echo "✅ Виртуальное окружение активировано"
else
    echo "❌ Виртуальное окружение не найдено"
fi

# Добавление ROS Python пакетов
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH
export PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/site-packages:$PYTHONPATH

echo "Python: $(python3 --version)"
echo ""
echo "🚀 Теперь можно запускать ноду:"
echo "cd ~/cvat/plavki/src/trafic_detection/trafic_detection"
echo "python3 traffic_detector.py"
