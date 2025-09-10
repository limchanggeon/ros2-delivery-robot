#!/usr/bin/env python3
"""
Simple GUI wrapper for the `ros2-delivery-robot` Getting Started tasks.
- Uses Tkinter so it's lightweight and likely available on Jetson.
- Runs the project's install/build/launch scripts and shows realtime output.

Usage:
  cd <project_root>
  python3 gui/getting_started_gui.py

Notes:
- Run this on the Jetson device (or a machine with ROS environment available).
- The GUI executes shell commands; review scripts before running.
"""

import os
import sys
import threading
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
DEFAULT_WORKDIR = ROOT

COMMANDS = {
    'check_system': 'bash check_project_status.sh',
    'install_deps': 'bash install_python_deps.sh',
    'build_jetson': 'bash build_and_run_jetson.sh',
    'quick_fix_launch': 'bash quick_fix_launch.sh',
    'fix_launch_files': 'bash fix_launch_files_jetson.sh',
}

class Runner(threading.Thread):
    def __init__(self, cmd, workdir, output_callback):
        super().__init__(daemon=True)
        self.cmd = cmd
        self.workdir = workdir
        self.output_callback = output_callback
        self.process = None

    def run(self):
        self.output_callback(f"$ {self.cmd}\n")
        try:
            # Use a shell so project scripts and environment work as expected
            self.process = subprocess.Popen(
                self.cmd,
                cwd=self.workdir,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                executable='/bin/bash'
            )
            for line in self.process.stdout:
                self.output_callback(line)
            self.process.wait()
            self.output_callback(f"\n[PROCESS EXITED] code={self.process.returncode}\n")
        except Exception as e:
            self.output_callback(f"[ERROR] {e}\n")

    def terminate(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('ROS2 Delivery Robot — Quick Setup GUI')
        self.geometry('900x700')

        self.workdir = tk.StringVar(value=DEFAULT_WORKDIR)
        self.use_rviz = tk.BooleanVar(value=False)

        self._make_widgets()
        self.current_runner = None

    def _make_widgets(self):
        frm_top = ttk.Frame(self)
        frm_top.pack(fill='x', padx=8, pady=6)

        ttk.Label(frm_top, text='Project root:').pack(side='left')
        ttk.Entry(frm_top, textvariable=self.workdir, width=70).pack(side='left', padx=6)
        ttk.Button(frm_top, text='Browse', command=self._browse).pack(side='left')

        frm_buttons = ttk.Frame(self)
        frm_buttons.pack(fill='x', padx=8, pady=6)

        # Row 1
        ttk.Button(frm_buttons, text='Check System', command=self._check_system).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(frm_buttons, text='Install Python Deps', command=self._install_deps).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(frm_buttons, text='Build (Jetson)', command=self._build_jetson).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(frm_buttons, text='Quick Fix Launch', command=self._quick_fix_launch).grid(row=0, column=3, padx=4, pady=4)
        ttk.Button(frm_buttons, text='Fix Launch Files', command=self._fix_launch_files).grid(row=0, column=4, padx=4, pady=4)

        # Row 2
        ttk.Checkbutton(frm_buttons, text='use_rviz', variable=self.use_rviz).grid(row=1, column=0, padx=4, pady=4)
        ttk.Button(frm_buttons, text='Launch Full System', command=self._launch_full_system).grid(row=1, column=1, padx=4, pady=4)
        ttk.Button(frm_buttons, text='Run system_monitor_node', command=self._run_system_monitor).grid(row=1, column=2, padx=4, pady=4)
        ttk.Button(frm_buttons, text='Open README', command=self._open_readme).grid(row=1, column=3, padx=4, pady=4)
        ttk.Button(frm_buttons, text='Stop Current', command=self._stop_current).grid(row=1, column=4, padx=4, pady=4)

        # Output
        frm_out = ttk.Frame(self)
        frm_out.pack(fill='both', expand=True, padx=8, pady=6)
        self.txt = tk.Text(frm_out, wrap='none')
        self.txt.pack(side='left', fill='both', expand=True)
        self.txt.configure(state='disabled')

        vsb = ttk.Scrollbar(frm_out, orient='vertical', command=self.txt.yview)
        vsb.pack(side='right', fill='y')
        self.txt['yscrollcommand'] = vsb.set

    def _browse(self):
        d = filedialog.askdirectory(initialdir=self.workdir.get())
        if d:
            self.workdir.set(d)

    def _append(self, text):
        self.txt.configure(state='normal')
        self.txt.insert('end', text)
        self.txt.see('end')
        self.txt.configure(state='disabled')

    def _run_command(self, cmd, confirm=True):
        if confirm and not messagebox.askyesno('Confirm', f'Execute:\n{cmd}\n\nWorking dir: {self.workdir.get()}'):
            return
        if self.current_runner and self.current_runner.is_alive():
            messagebox.showwarning('Busy', 'A command is already running. Stop it first.')
            return
        self.current_runner = Runner(cmd, self.workdir.get(), self._append)
        self.current_runner.start()

    # Actions
    def _check_system(self):
        self._run_command(COMMANDS['check_system'])

    def _install_deps(self):
        self._run_command(COMMANDS['install_deps'])

    def _build_jetson(self):
        self._run_command(COMMANDS['build_jetson'])

    def _quick_fix_launch(self):
        self._run_command(COMMANDS['quick_fix_launch'])

    def _fix_launch_files(self):
        self._run_command(COMMANDS['fix_launch_files'])

    def _launch_full_system(self):
        # use_rviz option
        cmd = 'ros2 launch delivery_robot_mission full_system_launch.py'
        if not self.use_rviz.get():
            cmd += ' use_rviz:=false'
        self._run_command(cmd)

    def _run_system_monitor(self):
        self._run_command('ros2 run delivery_robot_mission system_monitor_node')

    def _open_readme(self):
        path = os.path.join(self.workdir.get(), 'GETTING_STARTED_JETSON.md')
        if os.path.isfile(path):
            # open with default pager (xdg-open) or just display
            try:
                subprocess.Popen(['xdg-open', path])
            except Exception:
                self._run_command(f'bat "{path}" || cat "{path}"', confirm=False)
        else:
            messagebox.showinfo('Not found', f'{path} does not exist')

    def _stop_current(self):
        if self.current_runner and self.current_runner.is_alive():
            if messagebox.askyesno('Stop', 'Terminate the running process?'):
                try:
                    self.current_runner.terminate()
                except Exception as e:
                    messagebox.showerror('Error', str(e))
        else:
            messagebox.showinfo('Idle', 'No running process')


def main():
    app = App()
    app.mainloop()

if __name__ == '__main__':
    main()
