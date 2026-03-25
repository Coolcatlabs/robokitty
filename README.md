# Robokitty
RoboKitty - Quadruped IK Walking Controller

12-DOF (3 per leg) inverse kinematics with walk/trot/pace gaits.
AX-12A Dynamixel servos via half-duplex UART.

Leg layout (top view, front facing up):
    FL (8/10/0)     FR (11/9/7)
    RL (5/3/6)      RR (2/1/4)

Target: Raspberry Pi 4.

## 🛠 Setup & Installation

We recommend using a virtual environment to keep your global Python installation clean.

### 1. Create a Virtual Environment

```bash
python -m venv .venv
```

Activate it:
```
# On macOS/Linux:
source .venv/bin/activate
# On Windows (PowerShell):
.\.venv\Scripts\Activate.ps1
```

### 2. Install Dependencies
Once your environment is active, install robokitty and its development tools:

For standard usage:
```
pip install .
```

For contributors:
```
pip install -e ".[dev]"
```

### 3. Initialize Pre-commit
We use pre-commit to maintain code quality. These hooks run automatically every time you try to git commit.

Install the git hooks:
```
pre-commit install
```

(Optional) Run against all files manually to check existing code:
```
pre-commit run --all-files
```

## 🚀 Usage
Confirm everything is working by calling the helper directly from your terminal:

```
robokitty --help
```

If connected to the device the following commands are available from the command line:
```
robokitty              # Normal walk mode
robokitty --stand      # Stand only (calibration)
robokitty --diag       # IK diagnostics
robokitty --identify    # Flash servo LEDs
robokitty --read-pose  # Read positions (torque off)
robokitty --calibrate  # Pose by hand, compute offsets
```
