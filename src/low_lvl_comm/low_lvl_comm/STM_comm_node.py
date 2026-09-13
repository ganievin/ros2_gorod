#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

import serial

from city_interfaces.action import MoveCommand


class STMComm(Node):
    """
    Мост между ROS2 и STM32 через USB-serial.

    Функции:
      A) Action-сервер /move_command (тип city_interfaces/action/MoveCommand).
         Получает атомарную команду TURN или GO_STRAIGHT, шлёт её в serial,
         дожидается ответа "... DONE" от STM и завершает goal.

      B) Пассивный reader-thread. Публикует одометрию из строк "x,y,theta"
         в /odom и распознаёт строки "TURN DONE" / "GO_STRAIGHT DONE",
         которые снимают ожидание в активном action-голу.
    """

    ODOM_RE = r'^-?\d+\.?\d*,-?\d+\.?\d*,-?\d+\.?\d*$'
    ODOM_PATTERN = None  # заполним в __init__ через re.compile
    DONE_TURN = 'TURN DONE'
    DONE_STRAIGHT = 'GO_STRAIGHT DONE'

    def __init__(self) -> None:
        super().__init__('STM_comm_node')

        # --- параметры ---
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('command_timeout_sec', 10.0)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.command_timeout = float(
            self.get_parameter('command_timeout_sec').value
        )

        # --- serial ---
        import re
        self._odom_re = re.compile(self.ODOM_RE)
        try:
            self._ser = serial.Serial(port, baudrate, timeout=0.2)
        except serial.SerialException as exc:
            self.get_logger().fatal(
                f'failed to open serial {port}: {exc}'
            )
            raise

        self.get_logger().info(f'serial {port}@{baudrate} opened')

        # --- синхронизация с reader-thread'ом ---
        self._ser_lock = threading.Lock()
        self._pending_kind: str = ''      # 'TURN' | 'GO_STRAIGHT' | ''
        self._pending_event = threading.Event()
        self._running = True

        # --- ROS-обвязка ---
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._action_server = ActionServer(
            self,
            MoveCommand,
            '/move_command',
            execute_callback=self._execute_goal,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
        )

        # --- reader-thread ---
        self._reader = threading.Thread(
            target=self._serial_read_loop,
            name='serial_reader',
            daemon=True,
        )
        self._reader.start()

        self.get_logger().info('STM_comm_node up')

    # --------------------------------------------------------------- action
    def _on_goal_request(self, goal_request) -> GoalResponse:
        cmd = goal_request.command_type
        if cmd not in ('TURN', 'GO_STRAIGHT'):
            self.get_logger().warn(f'reject: unknown command "{cmd}"')
            return GoalResponse.REJECT
        if cmd == 'TURN' and goal_request.direction not in ('LEFT', 'RIGHT'):
            self.get_logger().warn(
                f'reject: invalid TURN direction "{goal_request.direction}"'
            )
            return GoalResponse.REJECT
        if cmd == 'GO_STRAIGHT' and goal_request.distance_mm <= 0:
            self.get_logger().warn(
                f'reject: non-positive distance {goal_request.distance_mm}'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle):
        goal = goal_handle.request
        result = MoveCommand.Result()

        line = self._build_serial_line(goal)
        if line is None:
            self.get_logger().error('cannot build serial line — aborting')
            goal_handle.abort()
            result.success = False
            return result

        # Готовим ожидание ДО write, чтобы не потерять DONE, если STM
        # ответит очень быстро.
        self._pending_kind = goal.command_type
        self._pending_event.clear()

        try:
            with self._ser_lock:
                self._ser.write(line.encode('ascii'))
                self._ser.flush()
        except serial.SerialException as exc:
            self.get_logger().error(f'serial write failed: {exc}')
            self._pending_kind = ''
            goal_handle.abort()
            result.success = False
            return result

        self.get_logger().info(f'→ STM: {line.strip()}')

        # Ждём DONE от reader-thread'а.
        done = self._pending_event.wait(timeout=self.command_timeout)
        self._pending_kind = ''

        if not done:
            self.get_logger().error(
                f'command "{line.strip()}" timed out '
                f'after {self.command_timeout} s — aborting'
            )
            goal_handle.abort()
            result.success = False
            return result

        if goal_handle.is_cancel_requested:
            self.get_logger().warn('goal cancelled by client')
            goal_handle.canceled()
            result.success = False
            return result

        goal_handle.succeed()
        result.success = True
        return result

    @staticmethod
    def _build_serial_line(goal):
        if goal.command_type == 'TURN':
            return f'TURN {goal.direction}\n'
        if goal.command_type == 'GO_STRAIGHT':
            return f'GO_STRAIGHT {int(goal.distance_mm)}\n'
        return None

    # --------------------------------------------------------------- serial
    def _serial_read_loop(self) -> None:
        buf = ''
        while self._running and rclpy.ok():
            try:
                # Читаем любые доступные байты; timeout=0.2 задан в Serial().
                with self._ser_lock:
                    data = self._ser.read(self._ser.in_waiting or 1)
            except serial.SerialException as exc:
                self.get_logger().error(f'serial read error: {exc}')
                time.sleep(0.5)
                continue
            except Exception as exc:
                self.get_logger().error(f'read loop exception: {exc}')
                time.sleep(0.5)
                continue

            if not data:
                continue

            try:
                buf += data.decode('utf-8', errors='ignore')
            except Exception:
                # errors='ignore' уже стоит, но на всякий случай
                continue

            while '\n' in buf:
                raw_line, buf = buf.split('\n', 1)
                self._process_line(raw_line.strip())

    def _process_line(self, line: str) -> None:
        if not line:
            return

        # 1) Одометрия
        if self._odom_re.match(line):
            self._publish_odom(line)
            return

        # 2) Результаты команд
        if line == self.DONE_TURN:
            self._handle_done('TURN', line)
            return
        if line == self.DONE_STRAIGHT:
            self._handle_done('GO_STRAIGHT', line)
            return

        # 3) Всё остальное — в лог
        self.get_logger().warn(f'unknown serial line: {line!r}')

    def _handle_done(self, kind: str, raw: str) -> None:
        if self._pending_kind == kind:
            self.get_logger().info(f'← STM: {raw}')
            self._pending_event.set()
        elif self._pending_kind == '':
            # DONE без активного action'а — STM шлёт что-то не по делу
            self.get_logger().warn(
                f'got {raw!r} while no command is pending — ignored'
            )
        else:
            self.get_logger().warn(
                f'got {raw!r} while waiting for '
                f'{self._pending_kind} DONE — ignored'
            )

    def _publish_odom(self, line: str) -> None:
        try:
            xs, ys, ts = line.split(',')
            x = float(xs)
            y = float(ys)
            theta = float(ts)
        except ValueError as exc:
            self.get_logger().warn(f'odom parse error: {exc} on {line!r}')
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self._euler_to_quaternion(0.0, 0.0, theta)

        self._odom_pub.publish(msg)
        self.get_logger().debug(
            f'odom x={x:.3f} y={y:.3f} θ={theta:.3f}'
        )

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    # --------------------------------------------------------------- shutdown
    def destroy_node(self) -> bool:
        self._running = False
        # разбудим reader-thread на случай, если он висит в read()
        try:
            if self._reader.is_alive():
                self._reader.join(timeout=1.0)
        except Exception:
            pass
        try:
            if hasattr(self, '_ser') and self._ser.is_open:
                self._ser.close()
                self.get_logger().info('serial port closed')
        except Exception as exc:
            self.get_logger().warn(f'error closing serial: {exc}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = STMComm()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f'STM_comm_node crashed: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
