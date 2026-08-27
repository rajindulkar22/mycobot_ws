#!/usr/bin/env python3
"""Tkinter UI for the myCobot manipulation state machine."""

import argparse
import sys
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import remove_ros_args

from mycobot_sim_projects.manipulation_state_machine import (
    TASK_FLOW,
    ManipulationStateMachine,
    TaskState,
)


COLOR_PENDING = "#e8e8e8"
COLOR_ACTIVE = "#ffd966"
COLOR_DONE = "#b6d7a8"
COLOR_FAILED = "#ea9999"
COLOR_RECOVERY = "#f9cb9c"


class ManipulationStateMachineUI:
    """Desktop panel to start, stop, and monitor the demo FSM."""

    def __init__(
        self,
        arm_duration: float,
        pause_duration: float,
    ) -> None:
        self._run_thread: threading.Thread | None = None

        rclpy.init()
        self._node = ManipulationStateMachine(
            arm_duration=arm_duration,
            pause_duration=pause_duration,
        )
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            daemon=True,
        )
        self._spin_thread.start()

        self.root = tk.Tk()
        self.root.title("myCobot Manipulation State Machine")
        self.root.geometry("760x520")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_widgets()
        self._poll_ui()

    def _build_widgets(self) -> None:
        header = ttk.Label(
            self.root,
            text="Demo sequence: home → open → ready → approach → "
            "close → lift → release → home",
            wraplength=720,
        )
        header.pack(padx=12, pady=(12, 6), anchor="w")

        note = ttk.Label(
            self.root,
            text=(
                "Requires Gazebo with arm_controller and "
                "gripper_action_controller running. "
                "For cube pick, use gazebo_pose_commander pick_cube instead."
            ),
            wraplength=720,
            foreground="#555555",
        )
        note.pack(padx=12, pady=(0, 10), anchor="w")

        flow_frame = ttk.LabelFrame(self.root, text="State flow")
        flow_frame.pack(fill="x", padx=12, pady=6)

        self._state_labels: dict[TaskState, tk.Label] = {}
        row = ttk.Frame(flow_frame)
        row.pack(padx=8, pady=8)

        for index, state in enumerate(TASK_FLOW):
            label = tk.Label(
                row,
                text=state.name.replace("_", "\n"),
                width=10,
                height=3,
                relief="groove",
                bg=COLOR_PENDING,
                font=("TkDefaultFont", 9, "bold"),
            )
            label.pack(side="left", padx=2)
            self._state_labels[state] = label

            if index < len(TASK_FLOW) - 1:
                ttk.Label(row, text="→").pack(side="left", padx=1)

        status_frame = ttk.LabelFrame(self.root, text="Current status")
        status_frame.pack(fill="x", padx=12, pady=6)

        self._status_var = tk.StringVar(value="IDLE")
        ttk.Label(
            status_frame,
            textvariable=self._status_var,
            font=("TkDefaultFont", 14, "bold"),
        ).pack(padx=10, pady=8, anchor="w")

        settings = ttk.LabelFrame(self.root, text="Timing")
        settings.pack(fill="x", padx=12, pady=6)

        settings_row = ttk.Frame(settings)
        settings_row.pack(padx=8, pady=8, anchor="w")

        ttk.Label(settings_row, text="Arm duration (s):").grid(
            row=0, column=0, sticky="w", padx=(0, 6)
        )
        self._arm_duration_var = tk.DoubleVar(
            value=self._node.arm_duration
        )
        self._arm_duration_spin = ttk.Spinbox(
            settings_row,
            from_=1.0,
            to=30.0,
            increment=0.5,
            width=8,
            textvariable=self._arm_duration_var,
            command=self._apply_timing,
        )
        self._arm_duration_spin.grid(row=0, column=1, padx=(0, 16))

        ttk.Label(settings_row, text="Pause (s):").grid(
            row=0, column=2, sticky="w", padx=(0, 6)
        )
        self._pause_var = tk.DoubleVar(value=self._node.pause_duration)
        self._pause_spin = ttk.Spinbox(
            settings_row,
            from_=0.0,
            to=5.0,
            increment=0.1,
            width=8,
            textvariable=self._pause_var,
            command=self._apply_timing,
        )
        self._pause_spin.grid(row=0, column=3)

        controls = ttk.Frame(self.root)
        controls.pack(fill="x", padx=12, pady=8)

        self._start_button = ttk.Button(
            controls,
            text="Start sequence",
            command=self._on_start,
        )
        self._start_button.pack(side="left", padx=(0, 8))

        self._stop_button = ttk.Button(
            controls,
            text="Stop",
            command=self._on_stop,
            state="disabled",
        )
        self._stop_button.pack(side="left", padx=(0, 8))

        self._reset_button = ttk.Button(
            controls,
            text="Reset",
            command=self._on_reset,
        )
        self._reset_button.pack(side="left")

        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.pack(fill="both", expand=True, padx=12, pady=(6, 12))

        self._log = tk.Text(log_frame, height=8, state="disabled")
        self._log.pack(fill="both", expand=True, padx=8, pady=8)

    def _apply_timing(self) -> None:
        if self._node.is_running:
            return

        try:
            self._node.arm_duration = float(self._arm_duration_var.get())
            self._node.pause_duration = float(self._pause_var.get())
        except tk.TclError:
            pass

    def _append_log(self, message: str) -> None:
        self._log.configure(state="normal")
        self._log.insert("end", message + "\n")
        self._log.see("end")
        self._log.configure(state="disabled")

    def _set_controls_running(self, running: bool) -> None:
        if running:
            self._start_button.configure(state="disabled")
            self._stop_button.configure(state="normal")
            self._reset_button.configure(state="disabled")
            self._arm_duration_spin.configure(state="disabled")
            self._pause_spin.configure(state="disabled")
        else:
            self._start_button.configure(state="normal")
            self._stop_button.configure(state="disabled")
            self._reset_button.configure(state="normal")
            self._arm_duration_spin.configure(state="normal")
            self._pause_spin.configure(state="normal")

    def _on_start(self) -> None:
        if self._node.is_running:
            return

        self._apply_timing()
        self._node.reset()
        self._append_log("Starting manipulation sequence ...")
        self._set_controls_running(True)

        self._run_thread = threading.Thread(
            target=self._run_sequence,
            daemon=True,
        )
        self._run_thread.start()

    def _run_sequence(self) -> None:
        success = self._node.run()
        self.root.after(
            0,
            lambda: self._on_sequence_finished(success),
        )

    def _on_sequence_finished(self, success: bool) -> None:
        if success:
            self._append_log("Sequence completed successfully.")
        else:
            self._append_log(
                f"Sequence failed: {self._node.status_label}"
            )
        self._set_controls_running(False)

    def _on_stop(self) -> None:
        if not self._node.is_running:
            return

        self._node.request_stop()
        self._append_log("Stop requested (finishes current step first) ...")

    def _on_reset(self) -> None:
        if self._node.is_running:
            messagebox.showwarning(
                "Running",
                "Stop the sequence before resetting.",
            )
            return

        self._node.reset()
        self._append_log("State machine reset to HOME.")
        self._refresh_state_colors()

    def _refresh_state_colors(self) -> None:
        current = self._node.state

        if current == TaskState.RECOVERY:
            for label in self._state_labels.values():
                label.configure(bg=COLOR_RECOVERY)
            return

        if current == TaskState.FAILED:
            for state, label in self._state_labels.items():
                if (
                    self._node.failed_state
                    and state.name == self._node.failed_state
                ):
                    label.configure(bg=COLOR_FAILED)
                else:
                    label.configure(bg=COLOR_PENDING)
            return

        try:
            current_index = TASK_FLOW.index(current)
        except ValueError:
            current_index = -1

        for index, state in enumerate(TASK_FLOW):
            label = self._state_labels[state]
            if index < current_index:
                label.configure(bg=COLOR_DONE)
            elif index == current_index:
                label.configure(bg=COLOR_ACTIVE)
            else:
                label.configure(bg=COLOR_PENDING)

    def _poll_ui(self) -> None:
        self._status_var.set(self._node.status_label)
        self._refresh_state_colors()
        self.root.after(200, self._poll_ui)

    def _on_close(self) -> None:
        if self._node.is_running:
            self._node.request_stop()

        self._executor.shutdown()
        self._node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="GUI for the myCobot manipulation state machine."
    )
    parser.add_argument(
        "--arm-duration",
        type=float,
        default=5.0,
        help="Duration of each arm move in seconds.",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=0.5,
        help="Pause between states in seconds.",
    )

    arguments = remove_ros_args(args=sys.argv)[1:]
    parsed = parser.parse_args(arguments)

    if parsed.arm_duration <= 0.0:
        parser.error("--arm-duration must be positive.")
    if parsed.pause < 0.0:
        parser.error("--pause cannot be negative.")

    return parsed


def main(args=None) -> None:
    parsed = parse_arguments()
    ui = ManipulationStateMachineUI(
        arm_duration=parsed.arm_duration,
        pause_duration=parsed.pause,
    )
    ui.run()


if __name__ == "__main__":
    main()
