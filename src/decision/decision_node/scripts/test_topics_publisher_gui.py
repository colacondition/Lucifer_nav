#!/usr/bin/env python3
"""手动发布决策节点输入话题的测试 GUI。

发布：
  - robot_status (RobotStatus): robot_id / current_hp / is_attacked
  - game_status  (GameStatus):  game_type / game_progress / stage_remain_time
订阅：
  - /decision/state (std_msgs/String): 实时回显当前决策状态

用途：
  - 让哨兵出门（game_progress=4 + 给它一个有意义的血量）
  - 拨 stage_remain_time 看中心走位换位置（它决定走位的激进度，见 README §4）
  - 扣血到低阈值以下，看它回家

注意 RobotStatus 里 shooter_heat / team_color 已经不再填充（那两路数据不在上行帧里），
所以本 GUI 也不发它们。想让"出门→到达中心→走位"这条链路跑通，还需要真实的定位与
航点 CSV（或者仿真）。
"""

import sys
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

try:
    from decision_interfaces.msg import GameStatus, RobotStatus
except ModuleNotFoundError:
    print(
        "Cannot import decision_interfaces. "
        "Please source this workspace first, e.g.: source install/setup.bash",
        file=sys.stderr,
    )
    sys.exit(1)


class TestTopicsPublisherGUI:
    def __init__(self) -> None:
        rclpy.init()
        self.node = Node("test_topics_publisher_gui")

        self._pub_robot_status = self.node.create_publisher(RobotStatus, "robot_status", 10)
        self._pub_game_status = self.node.create_publisher(GameStatus, "game_status", 10)

        # /decision/state 使用 reliable + transient_local，与决策节点一致，
        # 晚启动也能立即收到最近一次状态。
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._decision_state = "(未收到)"
        self._sub_decision_state = self.node.create_subscription(
            String, "/decision/state", self._on_decision_state, state_qos
        )

        self.root = tk.Tk()
        self.root.title("RM Test Topics Publisher")
        self.root.geometry("460x560")

        # RobotStatus 字段
        self.robot_id_var = tk.StringVar(value="7")
        self.current_hp_var = tk.StringVar(value="400")
        self.is_attacked_var = tk.BooleanVar(value=False)
        # 持续掉血：每帧（0.1s）扣 10 点血，模拟持续被打（会一路扣到低血触发撤退）。
        self.drain_hp_var = tk.BooleanVar(value=False)

        # GameStatus 字段
        self.game_progress_var = tk.StringVar(value="4")
        self.stage_remain_time_var = tk.StringVar(value="300")

        # 回显
        self.status_var = tk.StringVar(value="Ready")
        self.decision_state_var = tk.StringVar(value=self._decision_state)
        self._running = True

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # ROS spin 放在后台线程，Tk 主循环负责界面与定时发布。
        self._ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._ros_thread.start()
        self.root.after(100, self._tick)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        # ---- RobotStatus ----
        robot_frame = ttk.LabelFrame(main, text="RobotStatus", padding=8)
        robot_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(robot_frame, text="Robot ID").grid(row=0, column=0, sticky=tk.W, padx=(0, 8), pady=2)
        ttk.Entry(robot_frame, textvariable=self.robot_id_var, width=12).grid(row=0, column=1, sticky=tk.W, pady=2)

        ttk.Label(robot_frame, text="Current HP").grid(row=1, column=0, sticky=tk.W, padx=(0, 8), pady=2)
        ttk.Entry(robot_frame, textvariable=self.current_hp_var, width=12).grid(row=1, column=1, sticky=tk.W, pady=2)
        # 快捷掉血/加血按钮，方便制造 hp 变化率
        hp_btns = ttk.Frame(robot_frame)
        hp_btns.grid(row=1, column=2, sticky=tk.W, padx=(8, 0))
        ttk.Button(hp_btns, text="-50", width=4, command=lambda: self._bump_hp(-50)).pack(side=tk.LEFT)
        ttk.Button(hp_btns, text="+50", width=4, command=lambda: self._bump_hp(50)).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(hp_btns, text="重置400", width=6, command=lambda: self._set_hp(400)).pack(side=tk.LEFT, padx=(4, 0))

        ttk.Checkbutton(robot_frame, text="is_attacked (正在挨打)", variable=self.is_attacked_var).grid(
            row=2, column=0, columnspan=2, sticky=tk.W, pady=2
        )
        ttk.Checkbutton(
            robot_frame, text="持续掉血 (模拟被打)", variable=self.drain_hp_var
        ).grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=2)

        # ---- GameStatus ----
        game_frame = ttk.LabelFrame(main, text="GameStatus", padding=8)
        game_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(game_frame, text="Game Progress").grid(row=0, column=0, sticky=tk.W, padx=(0, 8), pady=2)
        ttk.Entry(game_frame, textvariable=self.game_progress_var, width=12).grid(row=0, column=1, sticky=tk.W, pady=2)
        ttk.Label(game_frame, text="Stage Remain Time").grid(row=1, column=0, sticky=tk.W, padx=(0, 8), pady=2)
        ttk.Entry(game_frame, textvariable=self.stage_remain_time_var, width=12).grid(
            row=1, column=1, sticky=tk.W, pady=2
        )

        # ---- 场景快捷键 ----
        scenario = ttk.LabelFrame(main, text="快捷场景", padding=8)
        scenario.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(scenario, text="平静", command=self._scene_calm).pack(side=tk.LEFT)
        ttk.Button(scenario, text="低血量(回家)", command=self._scene_low_hp).pack(side=tk.LEFT, padx=(6, 0))

        # ---- 回显 ----
        echo = ttk.LabelFrame(main, text="决策回显", padding=8)
        echo.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(echo, text="/decision/state:").grid(row=0, column=0, sticky=tk.W, padx=(0, 8))
        ttk.Label(echo, textvariable=self.decision_state_var, font=("TkDefaultFont", 11, "bold")).grid(
            row=0, column=1, sticky=tk.W
        )

        ttk.Label(main, textvariable=self.status_var).pack(anchor=tk.W, pady=(6, 0))

    # ---- 快捷操作 ----
    def _bump_hp(self, delta: int) -> None:
        try:
            hp = int(self.current_hp_var.get().strip())
        except ValueError:
            hp = 0
        self.current_hp_var.set(str(max(0, min(65535, hp + delta))))

    def _set_hp(self, hp: int) -> None:
        self.current_hp_var.set(str(max(0, min(65535, hp))))

    def _scene_calm(self) -> None:
        self.is_attacked_var.set(False)
        self.drain_hp_var.set(False)

    def _scene_low_hp(self) -> None:
        # 低阈值是 120（见 yaml 的 hp_recovery.low_threshold）：低于它就该回家。
        self.current_hp_var.set("80")

    # ---- ROS 回调 ----
    def _on_decision_state(self, msg: String) -> None:
        self._decision_state = msg.data

    def _spin_ros(self) -> None:
        try:
            rclpy.spin(self.node)
        except Exception:
            pass

    # ---- 定时发布 ----
    def _tick(self) -> None:
        if not self._running:
            return
        ok = self._publish()
        self.status_var.set(
            "[OK] Auto publishing robot_status + game_status" if ok else "[ERR] Invalid input"
        )
        self.decision_state_var.set(self._decision_state)
        self.root.after(100, self._tick)

    def _publish(self) -> bool:
        try:
            robot_id = int(self.robot_id_var.get().strip())
            current_hp = int(self.current_hp_var.get().strip())
            game_progress = int(self.game_progress_var.get().strip())
            stage_remain_time = int(self.stage_remain_time_var.get().strip())
        except ValueError:
            return False

        if self.drain_hp_var.get():
            # 持续掉血：每 0.1s 扣 10 点血，模拟持续被打。
            current_hp = max(0, current_hp - 10)
            self.current_hp_var.set(str(current_hp))

        robot_msg = RobotStatus()
        robot_msg.robot_id = max(0, min(255, robot_id))
        robot_msg.current_hp = max(0, min(65535, current_hp))
        robot_msg.is_attacked = bool(self.is_attacked_var.get())
        self._pub_robot_status.publish(robot_msg)

        game_msg = GameStatus()
        game_msg.game_progress = max(0, min(255, game_progress))
        game_msg.stage_remain_time = max(0, min(65535, stage_remain_time))
        self._pub_game_status.publish(game_msg)
        return True

    def _on_close(self) -> None:
        self._running = False
        if rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    app = TestTopicsPublisherGUI()
    app.run()


if __name__ == "__main__":
    main()
