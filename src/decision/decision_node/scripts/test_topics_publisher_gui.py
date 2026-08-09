#!/usr/bin/env python3

import sys
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

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

        self.root = tk.Tk()
        self.root.title("RM Test Topics Publisher")
        self.root.geometry("420x260")

        self.current_hp_var = tk.StringVar(value="400")
        self.game_progress_var = tk.StringVar(value="0")
        self.stage_remain_time_var = tk.StringVar(value="300")
        self.status_var = tk.StringVar(value="Ready")
        self._running = True

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._tick)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        hp_frame = ttk.LabelFrame(main, text="Current HP", padding=8)
        hp_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(hp_frame, text="Current HP").grid(row=0, column=0, sticky=tk.W, padx=(0, 8), pady=2)
        ttk.Entry(hp_frame, textvariable=self.current_hp_var, width=12).grid(row=0, column=1, sticky=tk.W, pady=2)

        game_frame = ttk.LabelFrame(main, text="Game Status", padding=8)
        game_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(game_frame, text="Game Progress").grid(row=0, column=0, sticky=tk.W, padx=(0, 8), pady=2)
        ttk.Entry(game_frame, textvariable=self.game_progress_var, width=12).grid(
            row=0, column=1, sticky=tk.W, pady=2
        )
        ttk.Label(game_frame, text="Stage Remain Time").grid(row=1, column=0, sticky=tk.W, padx=(0, 8), pady=2)
        ttk.Entry(game_frame, textvariable=self.stage_remain_time_var, width=12).grid(
            row=1, column=1, sticky=tk.W, pady=2
        )

        ttk.Label(main, textvariable=self.status_var).pack(anchor=tk.W, pady=(10, 0))

    def _tick(self) -> None:
        if not self._running:
            return
        ok = self._publish()
        if not ok:
            self.status_var.set("[ERR] Invalid input")
        self.root.after(100, self._tick)

    def _publish(self) -> None:
        try:
            current_hp = int(self.current_hp_var.get().strip())
            game_progress = int(self.game_progress_var.get().strip())
            stage_remain_time = int(self.stage_remain_time_var.get().strip())
        except ValueError:
            return False

        robot_msg = RobotStatus()
        robot_msg.robot_id = 7
        robot_msg.current_hp = max(0, min(65535, current_hp))
        robot_msg.shooter_heat = 0
        robot_msg.team_color = False
        robot_msg.is_attacked = False
        self._pub_robot_status.publish(robot_msg)

        game_msg = GameStatus()
        game_msg.game_progress = max(0, min(255, game_progress))
        game_msg.stage_remain_time = max(0, min(65535, stage_remain_time))
        self._pub_game_status.publish(game_msg)

        self.status_var.set("[OK] Auto publishing current_hp and game_status")
        return True

    def _on_close(self) -> None:
        self._running = False
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    app = TestTopicsPublisherGUI()
    app.run()


if __name__ == "__main__":
    main()
