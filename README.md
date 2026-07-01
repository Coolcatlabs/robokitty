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

### Gait control
```
robokitty gait          # default crawl
robokitty gait crawl
robokitty gait walk
```

### Servo tools
```
robokitty servo diagnose <id>   # full register diagnostic
robokitty servo repair <id>     # attempt software recovery
robokitty servo set-id          # safely change a servo's ID
robokitty servo jog <id> <angle>  # jog servo to angle and back

```

### Global options
```
robokitty --port /dev/ttyUSB0 --baud 1000000 gait crawl
```


## 🚀 How to Run the Simulation
To ensure Webots correctly inherits the root repository context and reads the webots.yml file, use one of the following methods to launch the project:

Method 1: Command Line (Recommended)
Navigate to the root of your cloned repository and launch the world file directly. This forces Webots to use the repository root as its active project directory:

```
cd sim
webots worlds/robokitty.wbt
```

### Method 2: Opening via Webots GUI
1. Open the Webots application.
2. Select **File** > **Open World...** from the top menu.
3. Navigate into the `worlds/` directory of this repo and select `robokitty.wbt`.

> ⚠️ **Note:** Do not use *File > Open Project Directory*. Webots automatically detects the project root when you open the `.wbt` file from this structure.
