# Control: PID Controller
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[sim_gif]: ./images/sim.gif "PID controller running in the simulator"
[sim_P_H_gif]: ./images/high_p.gif "P only controller running in the simulator, with high coefficient"
[sim_P_L_gif]: ./images/low_p.gif "P only controller running in the simulator, with low coefficient"
[sim_PI_H_gif]: ./images/high_i.gif "PI controller running in the simulator, with high I coefficient"
[sim_PD_H_gif]: ./images/high_d.gif "PD controller running in the simulator, with high D coefficient"
[sim_PD_L_gif]: ./images/low_d.gif "PD controller running in the simulator, with low D coefficient"
[tuned_a]: ./images/tuned_a.png "CTE value over time with tuned parameters"
[tuned_a_fxd]: ./images/tuned_a_fixed.png "CTE value over time with fixed I"
[tuned_b]: ./images/tuned_b.png "CTE value over time for final tuned values"

![alt text][sim_gif]

Overview
---

This repository contains a C++ implementation of a proportional–integral–derivative (PID) controller that is used in order to direct a vehicle to follow a desired trajectory. A PID controller is a control loop feedback mechanism that is used in applications that requires a continuos modulated control.

In simple terms the idea behind a PID controller is to output a "corrective" value that is related to the current error that can be expressed in many forms, in the case of this project we use the cross track error (CTE) expressed as the *lateral* distance between the vehicle and the center of the target trajectory. The PID then outputs a correction amount (which in our case is the steering amount to apply in order to reach the target trajectory) that is correlated to 3 different error components:

- *Proportional*: The amount is directly related to the amount of CTE
- *Integral*: The amount is related to the cumulative CTE, in order to counter eventual systematic bias (that are evident over a longer period of time), for example a slight misalignment of the tires
- *Derivative*: The amount is related to the difference between the current and previous CTE value, with the intent of reducing the amount of "correction" to be applied the closer the vehicle is to the target

In this project the CTE value comes pre-computed from a simulated environment thanks to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim) and it's fed to the program through [WebSockets](https://en.wikipedia.org/wiki/WebSocket) messages. The [main](./src/main.cpp) file processes the incoming messages and parses the data that is then processed by the [PID](./src/PID.cpp) class.

The program includes some predefined values for the set of *coefficients* that are applied for each of the error components (e.g. in order to reduce or augment the effect of the single error component to the whole PID). Due to some limitation in the implementation of the simulator, these values may need to be adjusted according to the system the program runs on. The program accepts 3 (double) parameters in input Kp, Ki and Kd that corresponds to the proportional, integral and derivative coefficients respectively.

Additionally a simple [auto-tuning](./src/Tuner.cpp) mechanism was implemented that can be enabled providing as a 4th argument the number of steps to use for error collection before tuning the parameters. The tuning mechanism (implementation based on the [twiddle/coordinate ascent algorithm](https://www.youtube.com/watch?v=2uQ2BSzDvXs) from Sebastian Thrun) runs the simulation for several cycles (one cycle being composed by a max number of steps) and at each cycle the parameters are tuned according to the average (squared) error in order to steer towards a minima.

PID Coefficients
---

We can observe how the magnitude of the various coefficients affect the vehicle behavior:

#### Proportional (*Kp*): 

Given that the value is proportional to the amount of error its value is the easiest to observe. The higher the coefficient the fastest the vehicle reacts, this is due to the fact that a big CTE implies a big output. Unfortunately this is not enough to have a smooth drive as the vehicle will tent to always overshoot and correct in the next iteration (overshooting in the opposite direction).

![alt text][sim_P_H_gif]

A lower value will increase the time the oscillation occurs (and reducing the amount of overshooting) but on the other end the vehicle is less reactive to the error.

![alt text][sim_P_L_gif]

#### Integral (*Ki*):

The integral component uses the cumulative error to check if the application has a systematic bias. Accumulating the error over time will empathize this effect and so it can be used as a measure of correction. If the system is biased the vehicle will tend to follow a shifted trajectory. In our particular case I could not easily identify a bias in the vehicle, on the other end the track stays the same in the simulation so there is a sort of systematic bias (e.g. more turns to the left) that can be corrected using the integral component to help making more central turns.

#### Derivative (*Kp*): 

This component simply takes the difference between the current CTE and the previous one and applies a correction accordingly having the effect to reduce to amount of correction the more the target is close, contributing to a smoother experience while trying to reach the desired trajectory. 

An higher value has the effect to increase the reaction time and at the same time to counter the oscillation due to the Proportional component:

![alt text][sim_PD_H_gif]

A lower value improves the reaction time but may not counter the overshooting:

![alt text][sim_PD_L_gif]

Parameter Tuning
---

With the previous observations we can proceed tuning the various parameters, simply observing the vehicle behavior.

There are various methodologies in the literature used to [tune a PID controller](https://en.wikipedia.org/wiki/PID_controller#Loop_tuning) both manual and automatic. In this project I mainly used manual tuning directly from observing the vehicle behavior to get to a stable point and later on fine-tuned the coefficients using a simple implementation that runs the simulation several times tweaking the various parameters at each cycle trying to find the combination that yields the lower average (squared) error. Moreover I also [plotted](./extra/cte_visualization.ipynb) the output of the CTE over time to get a better understanding of the effect of the various parameters.

I started out tuning the P component first, to get to a decent reaction time and oscillation around the desired trajectory without too much overshooting. Once I found a good value I moved on directly to the D component in order to counter the oscillation and overshooting issue, while still being able to face the turns. I decided to avoid touching the I component at this time as I didn't notice a particular bias. I then started the automatic tuning for several cycles to fine tune the parameters (still excluding the integral component).

|    | Initial | Tuned    |
|----|---------|----------|
| Kp | 0.2     | 0.260047 |
| Ki | 0.0     | 0.0      |
| Kp | 5.2     | 5.51264  |

After the first tuning attempt I performed a run using the tuned parameter recording the CTE values over time:

![alt text][tuned_a]

I noticed a certain amount of bias, which could be explained with either some systematic bias in the vehicle or simply the bias due to the track being mostly with left turns, at this point I added the integral component trying to see if it would improve:

![alt text][tuned_a_fxd]

Unfortunately the automatic tuning is sensible to get stuck in local optima and the architecture of the system does not help as a single cycle takes several seconds to run. Moreover the way I implemented the tuning is not great as the average error is not a sensible indicator of how smooth the vehicle drives (e.g. very quick turns around the center may yeild lower error than a smoothing driving but a bit shifted from the trajectory).

At this point I decided to re-tune manually but with a very low integral component to avoid having drastic effects on the vechicle. And then run the automatic tuner to fine-tune the final parameters (a recording of the run can be found [here](./images/sim.mp4)):

|    | Initial | Tuned      |
|----|---------|------------|
| Kp | 0.15    | 0.226576   |
| Ki | 0.0001  | 0.00011891 |
| Kp | 4.5     | 4.455      |

![alt text][tuned_b]

In the final parameters we can notice a slighlty better error graph with lower spikes and smoother curves while retaining a bit of bias. Note that this shows the limitation of the auto tuning implementation as there are multiple values that may lead to a "local" optima according to the tuner.

Getting Started
---

In order to run the program you need the simulator provided by [Udacity](https://www.udacity.com/) which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even better [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. The version compatible with the simulator is the uWebSocketIO branch **e94b6e1**.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. ```mkdir build```
2. ```cd build```
3. ```cmake .. && make```
4. ```./pid```

Note that to compile the program with debug symbols you can supply the appropriate flag to cmake: ```cmake -DCMAKE_BUILD_TYPE=Debug .. && make```.

Now the Udacity simulator can be run selecting the PID Control project, press start and see the application in action.

#### Other Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

Environment Setup
---

This project was developed under windows using the windows subsystem for linux ([WSL](https://docs.microsoft.com/en-us/windows/wsl/install-win10)) with Ubuntu Bash 16.04 together with [Visual Studio Code](https://code.visualstudio.com/).

The steps to setup the environment under mac, linux or windows (WSL) are more or less the same:

- Review the above dependencies
- Clone the repo and run the appropriate script (./install-ubuntu.sh under WSL and linux and ./install-mac.sh under mac), this should install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) from the branch **e94b6e1**

Under windows (WSL) and linux you can make a clean installation as follows:

1. ```sudo apt-get update```
2. ```sudo apt-get install git```
3. ```sudo apt-get install cmake```
4. ```sudo apt-get install openssl```
5. ```sudo apt-get install libssl-dev```
6. ```git clone https://github.com/marcomarasca/CarND-PID-Controller```
7. ```sudo rm /usr/lib/libuWS.so```
8. ```./install-ubuntu.sh```

#### Debugging with VS Code

Since I developed this project using WSL and Visual Studio Code it was very useful for me to setup a debugging pipeline. VS Code comes with a official Microsoft cpp extension that can be downloaded directly from the marketplace: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools. After the installation there are a few things to setup in order to make it work with the subsystem for linux, personally I went with the default Ubuntu distribution.

For the following setup I assume that the repository was cloned in **D:/Dev/CarND-PID-Controller/**.

##### Setup the language server (for IntelliSense)

From the official documentation [https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/Windows%20Subsystem%20for%20Linux.md](https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/Windows%20Subsystem%20for%20Linux.md): 

Simply Crtl+P and select "C/Cpp: Edit Configurations", this will create a c_cpp_properties.json file that can be configured as follows:

```json
{
    "name": "WSL",
    "intelliSenseMode": "clang-x64",
    "compilerPath": "/usr/bin/gcc",
    "includePath": [
        "${workspaceFolder}"
    ],
    "defines": [],
    "browse": {
        "path": [
            "${workspaceFolder}"
        ],
        "limitSymbolsToIncludedHeaders": true,
        "databaseFilename": ""
    },
    "cStandard": "c11",
    "cppStandard": "c++17"
}
```

##### Setup the Debugger

From the official documentation [https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/Debugger/gdb/Windows%20Subsystem%20for%20Linux.md](https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/Debugger/gdb/Windows%20Subsystem%20for%20Linux.md):

First install gdb in the WSL:

```
sudo apt install gdb
```

Then simply create a lunch configuration from VS Code: "Debug" -> "Add Configuration.." and setup the launch.json as follows:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "C++ Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "/mnt/d/Dev/CarND-PID-Controller/build/pid",
            "args": ["-fThreading"],
            "stopAtEntry": false,
            "cwd": "/mnt/d/Dev/CarND-Controller/build/",
            "environment": [],
            "externalConsole": true,
            "windows": {
                "MIMode": "gdb",
                "setupCommands": [
                    {
                        "description": "Enable pretty-printing for gdb",
                        "text": "-enable-pretty-printing",
                        "ignoreFailures": true
                    }
                ]
            },
            "pipeTransport": {
                "pipeCwd": "",
                "pipeProgram": "c:\\windows\\sysnative\\bash.exe",
                "pipeArgs": ["-c"],
                "debuggerPath": "/usr/bin/gdb"
            },
            "sourceFileMap": {
                "/mnt/d": "d:\\"
            }
        }
    ]
}
```

Note how the program is mapped directly into the file system of the WSL and piped through bash.exe (the paths are relative to the WSL environment).

Now you are ready to debug the application directly from VS Code, simply compile the application from within the WSL with the debug symbols:

```cmake -DCMAKE_BUILD_TYPE=Debug .. && make```

And run the debugger from VS Code (e.g. F5) :)
