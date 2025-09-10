# Getting Started GUI for ros2-delivery-robot

This lightweight Tkinter GUI provides convenient buttons to run the project's common setup and debugging scripts on the Jetson device.

## Features
- Run `check_project_status.sh`, `install_python_deps.sh`, `build_and_run_jetson.sh`, and quick-fix scripts
- Launch the full system (with or without RViz)
- Execute `system_monitor_node`
- View realtime stdout/stderr output in the GUI
- Stop a running command

## Usage
1. Install prerequisites (on Jetson):
```bash
sudo apt update
sudo apt install -y python3-tk
```

2. Start the GUI from the project root:
```bash
python3 gui/getting_started_gui.py
```

3. Use the buttons. Be cautious — the GUI runs shell commands that may modify your system.

## Security
- The GUI runs arbitrary shell commands specified in the script. Review `gui/getting_started_gui.py` before running.

## Troubleshooting
- If a button appears unresponsive, check the GUI output area for errors.
- For long-running builds, watch system resources (`top`, `jtop`).
