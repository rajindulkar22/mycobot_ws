#!/usr/bin/env python3
"""Tkinter workbench to launch Gazebo, MoveIt, and cube_approach pick-and-place."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import messagebox, scrolledtext, ttk
COLOR_STOPPED = "#e8e8e8"
COLOR_RUNNING = "#b6d7a8"
COLOR_FAILED = "#ea9999"


@dataclass
class ManagedProcess:
    """Long-running subprocess started in its own session."""

    name: str
    command: str
    popen: subprocess.Popen[str] | None = None
    status: str = "stopped"
    reader: threading.Thread | None = field(default=None, repr=False)


class SimWorkbenchUI:
    """Desktop panel to start the sim stack and run MoveIt pick-and-place."""

    PLACE_PRESETS: dict[str, tuple[float, float, float]] = {
        "Default": (0.10, 0.15, 0.140),
        "Left": (0.12, 0.18, 0.140),
        "Near": (0.15, 0.08, 0.140),
    }

    STACK_SERVICES: tuple[tuple[str, str], ...] = (
        (
            "gazebo",
            "ros2 launch mycobot_280jn_sim gazebo_sim.launch.py",
        ),
        (
            "move_group",
            "ros2 launch mycobot_280jn_moveit_config gazebo_move_group.launch.py",
        ),
        (
            "rviz",
            "ros2 launch mycobot_280jn_moveit_config gazebo_moveit_rviz.launch.py",
        ),
    )

    SIM_STACK_COMMAND = (
        "ros2 launch mycobot_280jn_moveit_config gazebo_moveit_stack.launch.py"
    )

    def __init__(self, workspace: str) -> None:
        self._workspace = os.path.abspath(os.path.expanduser(workspace))
        self._setup_script = os.path.join(
            self._workspace,
            "install",
            "setup.bash",
        )
        self._processes: dict[str, ManagedProcess] = {}
        self._pick_running = False
        self._status_labels: dict[str, tk.Label] = {}

        self.root = tk.Tk()
        self.root.title("myCobot Sim Workbench")
        self.root.geometry("820x680")
        self.root.minsize(720, 600)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_widgets()
        self._poll_processes()

    def _shell_command(self, command: str) -> list[str]:
        if not os.path.isfile(self._setup_script):
            raise FileNotFoundError(
                f"Workspace setup not found: {self._setup_script}"
            )

        script = (
            f"source {self._setup_script} && "
            f"cd {self._workspace} && "
            f"{command}"
        )
        return ["bash", "-lc", script]

    def _append_log(self, message: str) -> None:
        def write() -> None:
            self._log.configure(state="normal")
            self._log.insert("end", message + "\n")
            self._log.see("end")
            self._log.configure(state="disabled")

        self.root.after(0, write)

    def _set_status(self, name: str, status: str) -> None:
        label = self._status_labels.get(name)
        if label is None:
            return

        colors = {
            "stopped": COLOR_STOPPED,
            "running": COLOR_RUNNING,
            "failed": COLOR_FAILED,
        }
        label.configure(
            text=status.upper(),
            bg=colors.get(status, COLOR_STOPPED),
        )

    def _build_widgets(self) -> None:
        header = ttk.Label(
            self.root,
            text=(
                "Use 'Start Gazebo + MoveIt' (one process, synced sim clock). "
                "Then: open gripper → go home → run pick. Avoid starting "
                "move_group in a separate terminal after Gazebo restarts."
            ),
            wraplength=780,
        )
        header.pack(padx=12, pady=(12, 6), anchor="w")

        ws_label = ttk.Label(
            self.root,
            text=f"Workspace: {self._workspace}",
            foreground="#555555",
        )
        ws_label.pack(padx=12, pady=(0, 8), anchor="w")

        stack_frame = ttk.LabelFrame(self.root, text="Simulation stack")
        stack_frame.pack(fill="x", padx=12, pady=6)

        stack_btn_row = ttk.Frame(stack_frame)
        stack_btn_row.pack(fill="x", padx=8, pady=(8, 4))

        ttk.Button(
            stack_btn_row,
            text="Start Gazebo + MoveIt (recommended)",
            command=self._start_sim_stack,
        ).pack(side="left", padx=(0, 8))

        ttk.Button(
            stack_btn_row,
            text="Stop sim stack",
            command=self._stop_sim_stack,
        ).pack(side="left")

        ttk.Label(
            stack_btn_row,
            text="Prevents sim-time / TF jump errors",
            foreground="#666666",
        ).pack(side="left", padx=(12, 0))

        for index, (name, command) in enumerate(self.STACK_SERVICES):
            row = ttk.Frame(stack_frame)
            row.pack(fill="x", padx=8, pady=4)

            ttk.Label(row, text=name.replace("_", " ").title(), width=14).pack(
                side="left"
            )

            status = tk.Label(
                row,
                text="STOPPED",
                width=10,
                relief="groove",
                bg=COLOR_STOPPED,
                font=("TkDefaultFont", 9, "bold"),
            )
            status.pack(side="left", padx=(0, 8))
            self._status_labels[name] = status

            ttk.Button(
                row,
                text="Start",
                command=lambda n=name, c=command: self._start_service(n, c),
            ).pack(side="left", padx=2)

            ttk.Button(
                row,
                text="Stop",
                command=lambda n=name: self._stop_service(n),
            ).pack(side="left", padx=2)

            if index == 0:
                ttk.Label(
                    row,
                    text="(required)",
                    foreground="#666666",
                ).pack(side="left", padx=6)
            elif index == 1:
                ttk.Label(
                    row,
                    text="(required for MoveIt)",
                    foreground="#666666",
                ).pack(side="left", padx=6)

        prep_frame = ttk.LabelFrame(self.root, text="Prepare arm")
        prep_frame.pack(fill="x", padx=12, pady=6)

        prep_row = ttk.Frame(prep_frame)
        prep_row.pack(padx=8, pady=8, anchor="w")

        self._open_gripper_button = ttk.Button(
            prep_row,
            text="Open gripper",
            command=self._on_open_gripper,
        )
        self._open_gripper_button.pack(side="left", padx=(0, 8))

        self._home_button = ttk.Button(
            prep_row,
            text="Go home",
            command=self._on_go_home,
        )
        self._home_button.pack(side="left")

        place_frame = ttk.LabelFrame(self.root, text="Place target (world TCP)")
        place_frame.pack(fill="x", padx=12, pady=6)

        place_row = ttk.Frame(place_frame)
        place_row.pack(padx=8, pady=8, anchor="w")

        self._place_x = tk.DoubleVar(value=0.10)
        self._place_y = tk.DoubleVar(value=0.15)
        self._place_z = tk.DoubleVar(value=0.140)
        self._place_descend_z = tk.DoubleVar(value=0.055)

        for col, (label, var) in enumerate(
            (
                ("X (m):", self._place_x),
                ("Y (m):", self._place_y),
                ("Z (m):", self._place_z),
            )
        ):
            ttk.Label(place_row, text=label).grid(
                row=0,
                column=col * 2,
                sticky="w",
                padx=(0, 4),
            )
            ttk.Spinbox(
                place_row,
                from_=-0.50,
                to=0.50,
                increment=0.01,
                width=8,
                textvariable=var,
                format="%.3f",
            ).grid(row=0, column=col * 2 + 1, padx=(0, 12))

        descend_row = ttk.Frame(place_frame)
        descend_row.pack(padx=8, pady=(0, 4), anchor="w")

        ttk.Label(descend_row, text="Release Z (m):").pack(side="left", padx=(0, 4))
        ttk.Spinbox(
            descend_row,
            from_=-0.50,
            to=0.50,
            increment=0.01,
            width=8,
            textvariable=self._place_descend_z,
            format="%.3f",
        ).pack(side="left")

        preset_row = ttk.Frame(place_frame)
        preset_row.pack(padx=8, pady=(0, 8), anchor="w")

        ttk.Label(preset_row, text="Presets:").pack(side="left", padx=(0, 6))
        for preset_name in self.PLACE_PRESETS:
            ttk.Button(
                preset_row,
                text=preset_name,
                command=lambda n=preset_name: self._apply_preset(n),
            ).pack(side="left", padx=2)

        pick_frame = ttk.LabelFrame(self.root, text="Pick-and-place")
        pick_frame.pack(fill="x", padx=12, pady=6)

        pick_row = ttk.Frame(pick_frame)
        pick_row.pack(padx=8, pady=8, anchor="w")

        self._run_pick_button = ttk.Button(
            pick_row,
            text="Run cube_approach",
            command=self._on_run_cube_approach,
        )
        self._run_pick_button.pack(side="left")

        ttk.Label(
            pick_row,
            text="Requires Gazebo + move_group running, gripper open at start.",
            foreground="#555555",
        ).pack(side="left", padx=(12, 0))

        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.pack(fill="both", expand=True, padx=12, pady=(6, 12))

        self._log = scrolledtext.ScrolledText(
            log_frame,
            height=12,
            state="disabled",
            wrap="word",
        )
        self._log.pack(fill="both", expand=True, padx=8, pady=8)

        if not os.path.isfile(self._setup_script):
            self._append_log(
                f"WARNING: {self._setup_script} not found. "
                "Build the workspace before launching services."
            )

    def _apply_preset(self, preset_name: str) -> None:
        values = self.PLACE_PRESETS[preset_name]
        self._place_x.set(values[0])
        self._place_y.set(values[1])
        self._place_z.set(values[2])
        self._place_descend_z.set(0.055)
        self._append_log(
            f"Place preset '{preset_name}': "
            f"x={values[0]:.3f}, y={values[1]:.3f}, z={values[2]:.3f}, "
            f"release_z=0.055"
        )

    def _start_log_reader(
        self,
        process: subprocess.Popen[str],
        prefix: str,
    ) -> threading.Thread:
        def reader() -> None:
            assert process.stdout is not None
            for line in process.stdout:
                self._append_log(f"[{prefix}] {line.rstrip()}")

        thread = threading.Thread(target=reader, daemon=True)
        thread.start()
        return thread

    def _pkill_orphan_sim_processes(self) -> None:
        """Kill leftover Gazebo / move_group processes that break sim /clock sync."""
        for pattern in ("move_group", "gazebo", "gzserver", "gzclient"):
            subprocess.run(
                ["pkill", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        self._append_log(
            "Cleaned up orphan Gazebo / move_group processes (pkill)."
        )

    def _sim_stack_running(self) -> bool:
        managed = self._processes.get("sim_stack")
        return (
            managed is not None
            and managed.popen is not None
            and managed.popen.poll() is None
        )

    def _start_sim_stack(self) -> None:
        if self._sim_stack_running():
            messagebox.showinfo(
                "Already running",
                "Gazebo + MoveIt stack is already running.",
            )
            return

        for name in ("sim_stack", "gazebo", "move_group", "rviz"):
            self._stop_service(name)
        self._pkill_orphan_sim_processes()

        try:
            popen = subprocess.Popen(
                self._shell_command(self.SIM_STACK_COMMAND),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except FileNotFoundError as error:
            messagebox.showerror("Workspace error", str(error))
            self._append_log(f"ERROR: {error}")
            return
        except OSError as error:
            messagebox.showerror("Launch failed", str(error))
            self._append_log(f"ERROR starting sim stack: {error}")
            return

        managed = ManagedProcess(
            name="sim_stack",
            command=self.SIM_STACK_COMMAND,
            popen=popen,
        )
        managed.status = "running"
        managed.reader = self._start_log_reader(popen, "sim_stack")
        self._processes["sim_stack"] = managed
        self._set_status("gazebo", "running")
        self._set_status("move_group", "running")
        self._append_log("Started Gazebo + MoveIt (combined sim stack).")
        self._append_log(
            "Wait ~15 s for Gazebo + move_group before prepare / pick."
        )

    def _stop_sim_stack(self) -> None:
        self._stop_service("sim_stack")
        self._stop_all_services()
        self._pkill_orphan_sim_processes()
        for name in ("gazebo", "move_group", "rviz"):
            self._set_status(name, "stopped")
        self._append_log("Sim stack stopped.")

    def _start_service(self, name: str, command: str) -> None:
        if name == "move_group" and self._sim_stack_running():
            messagebox.showinfo(
                "Already running",
                "move_group is already running via the combined sim stack.",
            )
            return

        if name == "gazebo":
            self._stop_service("sim_stack")
            move_group = self._processes.get("move_group")
            if (
                move_group is not None
                and move_group.popen is not None
                and move_group.popen.poll() is None
            ):
                self._append_log(
                    "Stopping move_group — restart it after Gazebo is running."
                )
                self._stop_service("move_group")

        existing = self._processes.get(name)
        if existing and existing.popen and existing.popen.poll() is None:
            messagebox.showinfo(
                "Already running",
                f"{name} is already running.",
            )
            return

        try:
            popen = subprocess.Popen(
                self._shell_command(command),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except FileNotFoundError as error:
            messagebox.showerror("Workspace error", str(error))
            self._append_log(f"ERROR: {error}")
            return
        except OSError as error:
            messagebox.showerror("Launch failed", str(error))
            self._append_log(f"ERROR starting {name}: {error}")
            return

        managed = ManagedProcess(name=name, command=command, popen=popen)
        managed.status = "running"
        managed.reader = self._start_log_reader(popen, name)
        self._processes[name] = managed
        self._set_status(name, "running")
        self._append_log(f"Started {name}.")
        if name == "move_group":
            self._append_log(
                "Wait ~5 s for move_group to initialize before running pick."
            )
        if name == "gazebo":
            self._append_log(
                "Wait ~10 s for Gazebo + controllers, then start move_group."
            )

    def _stop_service(self, name: str) -> None:
        managed = self._processes.get(name)
        if managed is None or managed.popen is None:
            self._set_status(name, "stopped")
            return

        if managed.popen.poll() is not None:
            managed.status = "stopped"
            managed.popen = None
            self._set_status(name, "stopped")
            return

        try:
            os.killpg(managed.popen.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

        try:
            managed.popen.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(managed.popen.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            managed.popen.wait(timeout=2.0)

        managed.popen = None
        managed.status = "stopped"
        self._set_status(name, "stopped")
        self._append_log(f"Stopped {name}.")

    def _stop_all_services(self) -> None:
        for name, _command in self.STACK_SERVICES:
            self._stop_service(name)

    def _run_one_shot(
        self,
        label: str,
        command: str,
        timeout: float = 120.0,
    ) -> None:
        self._append_log(f"Running {label} ...")

        def worker() -> None:
            try:
                result = subprocess.run(
                    self._shell_command(command),
                    capture_output=True,
                    text=True,
                    timeout=timeout,
                )
            except FileNotFoundError as error:
                self._append_log(f"ERROR: {error}")
                self.root.after(0, self._set_prep_enabled, True)
                return
            except subprocess.TimeoutExpired:
                self._append_log(f"ERROR: {label} timed out after {timeout:.0f}s.")
                self.root.after(0, self._set_prep_enabled, True)
                return

            if result.stdout.strip():
                for line in result.stdout.strip().splitlines():
                    self._append_log(f"[{label}] {line}")
            if result.stderr.strip():
                for line in result.stderr.strip().splitlines():
                    self._append_log(f"[{label}] {line}")

            if result.returncode == 0:
                self._append_log(f"{label} completed.")
            else:
                self._append_log(
                    f"{label} failed (exit code {result.returncode})."
                )

            self.root.after(0, self._set_prep_enabled, True)

        self._set_prep_enabled(False)
        threading.Thread(target=worker, daemon=True).start()

    def _set_prep_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled and not self._pick_running else "disabled"
        self._open_gripper_button.configure(state=state)
        self._home_button.configure(state=state)
        if not self._pick_running:
            self._run_pick_button.configure(state="normal")

    def _on_open_gripper(self) -> None:
        self._run_one_shot(
            "open gripper",
            "ros2 run mycobot_sim_projects gripper_commander -- open",
            timeout=30.0,
        )

    def _on_go_home(self) -> None:
        self._run_one_shot(
            "go home",
            "ros2 run mycobot_sim_projects gazebo_pose_commander -- home "
            "&& sleep 6",
            timeout=90.0,
        )

    def _stack_ready(self) -> bool:
        if self._sim_stack_running():
            return True

        for required in ("gazebo", "move_group"):
            managed = self._processes.get(required)
            if (
                managed is None
                or managed.popen is None
                or managed.popen.poll() is not None
            ):
                return False
        return True

    def _on_run_cube_approach(self) -> None:
        if self._pick_running:
            return

        if not self._stack_ready():
            messagebox.showwarning(
                "Stack not ready",
                "Start Gazebo and move_group before running cube_approach.",
            )
            return

        try:
            place_x = float(self._place_x.get())
            place_y = float(self._place_y.get())
            place_z = float(self._place_z.get())
            place_descend_z = float(self._place_descend_z.get())
        except tk.TclError:
            messagebox.showerror("Invalid place target", "Enter numeric place values.")
            return

        command = (
            "sleep 3 && ros2 launch mycobot_moveit_projects cube_approach.launch.py "
            f"place_x:={place_x:.3f} "
            f"place_y:={place_y:.3f} "
            f"place_z:={place_z:.3f} "
            f"place_descend_z:={place_descend_z:.3f}"
        )

        self._pick_running = True
        self._run_pick_button.configure(state="disabled")
        self._set_prep_enabled(False)
        self._append_log(
            f"Launching cube_approach "
            f"(place x={place_x:.3f}, y={place_y:.3f}, z={place_z:.3f}, "
            f"release_z={place_descend_z:.3f}) ..."
        )

        def worker() -> None:
            try:
                result = subprocess.run(
                    self._shell_command(command),
                    capture_output=True,
                    text=True,
                )
            except FileNotFoundError as error:
                self._append_log(f"ERROR: {error}")
                self.root.after(0, self._on_pick_finished, False)
                return

            if result.stdout.strip():
                for line in result.stdout.strip().splitlines():
                    self._append_log(f"[cube_approach] {line}")
            if result.stderr.strip():
                for line in result.stderr.strip().splitlines():
                    self._append_log(f"[cube_approach] {line}")

            combined = (result.stdout or "") + (result.stderr or "")
            combined_lower = combined.lower()
            failure_markers = (
                "process has died",
                "approach execution failed",
                "execution failed for",
                "planning failed for",
            )
            success = result.returncode == 0 and not any(
                marker in combined_lower for marker in failure_markers
            )

            if success:
                self._append_log("cube_approach completed successfully.")
            else:
                self._append_log(
                    f"cube_approach failed (exit code {result.returncode})."
                )

            self.root.after(0, self._on_pick_finished, success)

        threading.Thread(target=worker, daemon=True).start()

    def _on_pick_finished(self, _success: bool) -> None:
        self._pick_running = False
        self._run_pick_button.configure(state="normal")
        self._set_prep_enabled(True)

    def _poll_processes(self) -> None:
        for name, managed in list(self._processes.items()):
            if managed.popen is None:
                continue

            return_code = managed.popen.poll()
            if return_code is None:
                continue

            if return_code == 0:
                self._append_log(f"{name} exited cleanly.")
                managed.status = "stopped"
                self._set_status(name, "stopped")
            else:
                self._append_log(
                    f"{name} exited with code {return_code}."
                )
                managed.status = "failed"
                self._set_status(name, "failed")

            if name == "sim_stack":
                stack_status = "stopped" if return_code == 0 else "failed"
                self._set_status("gazebo", stack_status)
                self._set_status("move_group", stack_status)

            managed.popen = None

        self.root.after(500, self._poll_processes)

    def _on_close(self) -> None:
        running = [
            name
            for name, managed in self._processes.items()
            if managed.popen is not None and managed.popen.poll() is None
        ]

        if running:
            stop_all = messagebox.askyesno(
                "Stop services?",
                "These services are still running:\n"
                + ", ".join(running)
                + "\n\nStop them and close the workbench?",
            )
            if not stop_all:
                return
            self._stop_sim_stack()

        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="GUI workbench for Gazebo, MoveIt, and cube_approach."
    )
    parser.add_argument(
        "--workspace",
        default=os.environ.get("MYCOBOT_WS", "~/mycobot_ws"),
        help="Path to the colcon workspace (default: ~/mycobot_ws).",
    )
    return parser.parse_args()


def main() -> None:
    parsed = parse_arguments()
    ui = SimWorkbenchUI(workspace=parsed.workspace)
    ui.run()


if __name__ == "__main__":
    main()
