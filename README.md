# Readme

## Description
This Python script performs a Model Predictive Control (MPC) simulation for vehicle lateral control using the CasADi framework. The main objective of this script is to compute optimal controls for a given vehicle's model while considering several constraints.

The script involves building a nonlinear programming problem, solving the optimization problem, shifting the horizon, and plotting the results.

## Dependencies
The script depends on the following Python libraries:
- CasADi
- Numpy
- Math
- Time
- Matplotlib

## Details
In the main section of the script, several parameters related to the vehicle and its constraints are defined. These include but are not limited to:
- Sampling time (T) 
- Prediction horizon (N)
- Maximum and minimum velocities (vx_max, vx_min, vy_max, vy_min)
- Maximum and minimum lateral positions (Y_max, Y_min)
- Maximum and minimum accelerations (ax_max, ax_min)
- Maximum and minimum steering angles (df_max, df_min)
- Vehicle's mass and inertia (m, Iz)
- The front and rear tire characteristics (Cf_0, Cr_0)

Following this, the script declares symbolic variables for the states and controls, as well as tire characteristics and cornering stiffness. The right-hand side (rhs) of the system dynamics is also defined.

An optimal control problem is then defined with an objective function to minimize the cost associated with deviating from the target state and control effort. Constraints are added to ensure that the state and control changes remain within predefined bounds.

The script uses CasADi's interface to the Ipopt solver to solve this optimal control problem.

## Simulation and Results
The script then simulates the vehicle's motion over a predefined time, using the computed optimal controls. The state history, control history, and time history are recorded.

After the simulation, the script plots the vehicle's longitudinal velocity, lateral position, longitudinal acceleration, and steering angle against time.

## Usage
You can run this script directly as follows:

```bash
python3 vehicle_lateral_control.py
```

This script is designed to be self-contained and does not take any command line arguments. However, you can modify the defined parameters directly in the script to suit your needs.

## Note
Please note that this script doesn't include the functionality to handle exceptional cases or errors that might occur during the execution, such as solver failing to find a solution, the system becoming unstable, etc. You may need to implement such error handling depending on your specific needs.

## 描述
这个Python脚本使用CasADi框架进行车辆侧向控制的模型预测控制（MPC）仿真。该脚本的主要目标是在考虑多个约束的情况下，计算给定车辆模型的最优控制。

该脚本包括构建一个非线性规划问题，解决优化问题，移动预测范围，并绘制结果。

## 依赖项
脚本依赖于以下Python库：
- CasADi
- Numpy
- Math
- Time
- Matplotlib

## 详情
在脚本的主要部分，定义了与车辆及其约束相关的多个参数。包括但不限于：
- 采样时间 (T)
- 预测范围 (N)
- 最大和最小速度 (vx_max, vx_min, vy_max, vy_min)
- 最大和最小横向位置 (Y_max, Y_min)
- 最大和最小加速度 (ax_max, ax_min)
- 最大和最小转向角 (df_max, df_min)
- 车辆的质量和转动惯量 (m, Iz)
- 前后轮胎的特性 (Cf_0, Cr_0)

接着，脚本声明了状态和控制的符号变量，以及轮胎特性和转向刚度。同时也定义了系统动力学的右手边 (rhs)。

然后，定义了一个最优控制问题，目标函数是最小化偏离目标状态和控制工作的代价。添加了约束，以确保状态和控制变化保持在预定义的边界内。

该脚本使用CasADi的Ipopt求解器接口来解决这个最优控制问题。

## 仿真和结果
脚本接着模拟了预定义时间内车辆的运动，使用计算出的最优控制。记录了状态历史、控制历史和时间历史。

在仿真后，脚本绘制了车辆的纵向速度、横向位置、纵向加速度和转向角随时间的变化。

## 使用方法
你可以直接运行这个脚本，如下所示：

```bash
python3 vehicle_lateral_control.py
```

这个脚本设计为自包含的，并且不接受任何命令行参数。然而，你可以直接在脚本中修改定义的参数，以适应你的需求。

## 注意
请注意，这个脚本没有包含处理执行过程中可能出现的异常或错误的功能，如求解器未能找到解、系统变得不稳定等。你可能需要根据你的具体需求实现这种错误处理。
