import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import subprocess
import re


class NetworkMonitorNode(Node):

    def __init__(self):
        super().__init__('network_monitor_node')

        # 无线接口做成 ROS 参数。
        # 如果以后 USB Wi-Fi 网卡接口名变化，可以启动时直接覆盖，
        # 不需要再修改源码。
        self.declare_parameter('interface', 'wlP1p1s0')
        self.interface = (
            self.get_parameter('interface')
            .get_parameter_value()
            .string_value
        )

        self.timer = self.create_timer(
            1.0,
            self.check_network
        )

        self.state_pub = self.create_publisher(
            String,
            '/adaptive/network_state',
            10
        )

        self.current_state = ""

        # 网络状态防抖
        self.candidate_state = None
        self.candidate_count = 0

        # 降级需要连续 3 次确认
        self.degrade_confirm_count = 3

        # 恢复需要连续 5 次确认
        self.recover_confirm_count = 5

        self.last_tx_packets = None
        self.last_tx_retries = None
        self.last_tx_failed = None

        self.last_target_mac = None

        self.get_logger().info(
            f"Network monitor started | interface={self.interface}"
        )

    def get_interface_mode(self):

        result = subprocess.check_output(
            [
                "iw",
                "dev",
                self.interface,
                "info"
            ],
            text=True
        )

        for line in result.splitlines():

            line = line.strip()

            if line.startswith("type "):
                return line.split()[1]

        raise RuntimeError(
            f"Cannot determine interface mode for {self.interface}"
        )

    def get_target_mac(self):

        mode = self.get_interface_mode()

        # -------------------------------------------------
        # managed 模式：
        # 小主机作为 Wi-Fi 客户端。
        # 目标 MAC 是当前连接 AP 的 MAC。
        # -------------------------------------------------

        if mode == "managed":

            result = subprocess.check_output(
                [
                    "iw",
                    "dev",
                    self.interface,
                    "link"
                ],
                text=True
            )

            match = re.search(
                r"Connected to ([0-9a-fA-F:]{17})",
                result
            )

            if match:
                return match.group(1).lower(), mode

            raise RuntimeError(
                "Wi-Fi interface is not connected to an AP"
            )

        # -------------------------------------------------
        # AP 模式：
        # 小主机自己作为热点。
        # station dump 中列出的就是连接进来的客户端。
        #
        # 比赛最终场景默认只有一台笔记本连接，
        # 因此先使用发现的第一个 station。
        # -------------------------------------------------

        elif mode == "AP":

            result = subprocess.check_output(
                [
                    "iw",
                    "dev",
                    self.interface,
                    "station",
                    "dump"
                ],
                text=True
            )

            match = re.search(
                r"Station ([0-9a-fA-F:]{17})",
                result
            )

            if match:
                return match.group(1).lower(), mode

            raise RuntimeError(
                "No Wi-Fi client connected to AP"
            )

        else:

            raise RuntimeError(
                f"Unsupported Wi-Fi mode: {mode}"
            )

    def reset_counters(self):

        self.last_tx_packets = None
        self.last_tx_retries = None
        self.last_tx_failed = None

    def judge_state(
        self,
        signal,
        tx_rate,
        retry_ratio
    ):

        signal_value = float(
            signal.split()[0]
        )

        tx_value = float(
            tx_rate.split()[0]
        )

        # 当前实验阶段的临时阈值。
        # 室外测试后再根据真实数据调整。
        if (
            signal_value > -60
            and tx_value > 50
        ):
            return "GOOD"

        elif (
            signal_value > -70
            and tx_value > 25
        ):
            return "MEDIUM"

        else:
            return "BAD"

    def state_level(self, state):
        levels = {
            "BAD": 0,
            "MEDIUM": 1,
            "GOOD": 2
        }

        return levels[state]

    def stabilize_state(self, raw_state):

        # 第一次获得有效状态，直接采用
        if self.current_state == "":
            self.current_state = raw_state
            self.candidate_state = None
            self.candidate_count = 0

            self.get_logger().info(
                f"Initial network state: {raw_state}"
            )

            return raw_state

        # 当前检测结果和稳定状态相同
        if raw_state == self.current_state:
            self.candidate_state = None
            self.candidate_count = 0

            return self.current_state

        # 出现新的候选状态
        if raw_state != self.candidate_state:
            self.candidate_state = raw_state
            self.candidate_count = 1
        else:
            self.candidate_count += 1

        current_level = self.state_level(
            self.current_state
        )

        candidate_level = self.state_level(
            self.candidate_state
        )

        # 网络变差：确认较快
        if candidate_level < current_level:
            required_count = self.degrade_confirm_count

        # 网络恢复：确认较慢
        else:
            required_count = self.recover_confirm_count

        self.get_logger().info(
            f"State candidate: "
            f"{self.candidate_state} "
            f"({self.candidate_count}/{required_count})"
        )

        # 达到确认次数，真正切换状态
        if self.candidate_count >= required_count:

            old_state = self.current_state
            self.current_state = self.candidate_state

            self.candidate_state = None
            self.candidate_count = 0

            self.get_logger().info(
                f"Stable network state changed: "
                f"{old_state} -> {self.current_state}"
            )

        return self.current_state

    def check_network(self):

        try:

            target_mac, mode = self.get_target_mac()

            # 如果目标发生变化，例如：
            # managed -> AP
            # 或 AP 下换了一台笔记本，
            # 旧累计计数器不能继续使用。
            if target_mac != self.last_target_mac:

                self.get_logger().info(
                    f"Wireless target: "
                    f"mode={mode} | "
                    f"MAC={target_mac}"
                )

                self.last_target_mac = target_mac
                self.reset_counters()

            cmd = [
                "iw",
                "dev",
                self.interface,
                "station",
                "get",
                target_mac
            ]

            result = subprocess.check_output(
                cmd,
                text=True
            )

            data = {}

            for line in result.splitlines():

                line = line.strip()

                if ":" in line:

                    key, value = line.split(
                        ":",
                        1
                    )

                    data[key.strip()] = value.strip()

            signal = data.get(
                "signal",
                "unknown"
            )

            tx_rate = data.get(
                "tx bitrate",
                "unknown"
            )

            tx_packets = int(
                data.get(
                    "tx packets",
                    0
                )
            )

            tx_retries = int(
                data.get(
                    "tx retries",
                    0
                )
            )

            tx_failed = int(
                data.get(
                    "tx failed",
                    0
                )
            )

            if self.last_tx_packets is not None:

                dp = tx_packets - self.last_tx_packets
                dr = tx_retries - self.last_tx_retries
                df = tx_failed - self.last_tx_failed

                if dp > 0:

                    retry_ratio = dr / dp
                    fail_ratio = df / dp

                else:

                    retry_ratio = 0.0
                    fail_ratio = 0.0

                self.get_logger().info(
                    f"Mode={mode} | "
                    f"Target={target_mac} | "
                    f"Signal={signal} | "
                    f"TX={tx_rate} | "
                    f"dPkt={dp} | "
                    f"dRetry={dr} | "
                    f"dFail={df} | "
                    f"RetryRatio={retry_ratio:.3f} | "
                    f"FailRatio={fail_ratio:.3f}"
                )

                raw_state = self.judge_state(
                    signal,
                    tx_rate,
                    retry_ratio
                )

                stable_state = self.stabilize_state(
                    raw_state
                )

                msg = String()
                msg.data = stable_state

                self.state_pub.publish(msg)

                self.get_logger().info(
                    f"Raw={raw_state} | "
                    f"Stable={stable_state}"
                )

            self.last_tx_packets = tx_packets
            self.last_tx_retries = tx_retries
            self.last_tx_failed = tx_failed

        except Exception as e:

            self.get_logger().warn(
                f"Network check failed: {e}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = NetworkMonitorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
