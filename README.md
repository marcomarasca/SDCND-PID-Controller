# Control: PID Controller
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[sim_gif]: ./images/sim.gif "PID controller running in the simulator"
[sim_P_H_gif]: ./images/simP_H.gif "P only controller running in the simulator, with high coefficient"
[sim_P_L_gif]: ./images/simP_L.gif "P only controller running in the simulator, with low coefficient"
[sim_PI_H_gif]: ./images/simPI_H.gif "PI controller running in the simulator, with high I coefficient"
[sim_PD_H_gif]: ./images/simPD_H.gif "PD controller running in the simulator, with high D coefficient"
[sim_PD_L_gif]: ./images/simPD_L.gif "PD controller running in the simulator, with low D coefficient"

![alt text][sim_gif]

Overview
---

This repository contains a C++ implementation of a proportional–integral–derivative (PID) controller that is used in order to direct a vehicle to follow a desired trajectory. A PID controller is a control loop feedback mechanism that is used in applications that requires a continuos modulated control.

In simple terms the idea behind a PID controller is to output a "corrective" value that is related to the current error that can be expressed in many forms, in the case of this project we use the cross track error (CTE) expressed as the *lateral* distance between the vehicle and the center of the target trajectory. The PID then outputs a correction amount (which in our case is the steering amount to apply in order to reach the target trajectory) that is correlated to 3 different error forms:

- *Proportional*: The amount is directly related to the amount of CTE
- *Integral*: The amount is related to the cumulative CTE, in order to counter eventual systematic biases (that are evident over a longer period of time), for example a slight misalignment of the tires
- *Derivative*: The amount is related to the difference between the current and previous CTE value, with the intent of reducing the amount of "correction" to be applied the less the CTE

In this project the CTE value comes pre-computed from a simulated environment thanks to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim) and it's fed to the program through [WebSockets](https://en.wikipedia.org/wiki/WebSocket) messages. The [main](./src/main.cpp) file processes the incoming messages and parses the data that is then processed by the [PID](./src/PID.cpp) class.

The program includes some predefined values for the set of *coefficients* that are applied for each of the error component (e.g. in order to reduce or augment the effect of the single error form to the whole PID). Due to some limitation in the implementation of the simulator, these values may need to be adjusted according to the system the program runs on. The program accepts 3 (double) parameters in input kP, kI and kD that corresponds to the proportional, integral and derivative coefficients respectively.

Additionally a simple [auto-tuning](./src/Tuner.cpp) mechanism was implemented that can be enabled providing as a 4th argument the number of steps to use for error collection before tuning the parameters. The tuning mechanism simply runs the simulator for the given number of steps collecting the total error and tuning each of the parameter slightly in order to find a good combination, after tuning the cycle is repeated again until a certain threshold is reached.

PID Coefficients and Tuning
---

We can observe how the magnitude of the various coefficients affect the vehicle behavior:

### Proportional (*kP*): 

Given that the value is proportional to the amount of error its value is easiest to observe. The higher the coefficient the fastest the vehicle reacts, this is due to the fact that a big CTE implies a big output. Unfortunately this is not enough to have a smooth drive as the vehicle will tent to always overshoot and correct in the next iteration (overshooting in the opposite direction). 

![alt text][sim_P_H_gif]

A lower value will increase the time the oscillation occurs (and reducing the amount of overshooting) but on the other end the vehicle is less reactive to the error.

![alt text][sim_P_L_gif]

### Integral (*kI*):

The integral component uses the cumulative error to check if the application has a systematic bias. Accumulating the error over time will empathize this effect and so it can be used as a measure of correction. If the system is biased the vehicle will tend to follow a shifted trajectory. In our particular case I could not identify a bias in the vehicle, and if present it's definitely too small too notice, on the other end the track stays the same in the simulation so there is a sort of systematic bias (e.g. more turns to the right) that can be corrected using the integral component.

In the following an example of a PI only controller using a very high I coefficient:

![alt text][sim_PI_H_gif]

### Derivative (*kP*): 

This component simply takes the difference between the current CTE and the previous one and applies a correction accordingly having the effect to reduce to amount of correction the more the target is close, contributing to a smoother experience while trying to reach the desired trajectory. 

An higher value has the effect to reduce the reaction time and at the same time to counter the oscillation due to the Proportional component:

![alt text][sim_PD_H_gif]

A lower value increases the reaction time but may not counter the overshooting:

![alt text][sim_PD_L_gif]

With this initial observations we can quickly proceed for a tuning of the various parameters, simply observing the vehicle behavior.

There are various methodologies in the literature used to [tune a PID controller](https://en.wikipedia.org/wiki/PID_controller#Loop_tuning) both manual and automatic. In this project I mainly used manual tuning directly from observing the vehicle behavior to get to a stable point and later on fine-tuned the coefficients using a simple implementation that runs the simulation several times tweaking the various parameters at each cycle trying to find the combination that yields the lower average (squared) error.

I started out tuning the P component first to get to a decent reaction time and oscillation around the desired trajectory without too much overshooting. Once I found a good value I moved on directly to the D component in order to counter the oscillation and overshooting issue, while still being able to face the turns. I decided to avoid touching the I component at this time as I didn't notice a particular bias. I then started the automatic tuning for several cycles to fine tune the parameters. Unfortunately the automatic tuning is sensible to get stuck in local optima and the architecture of the system does not help as a single simulation takes several seconds to run.

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

![alt text][sim_gif]

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
6. ```git clone https://github.com/Az4z3l/CarND-PID-Controller```
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
