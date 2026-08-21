# micro-ROS STM32 + BNO055 IMU

Firmware for an STM32F401RE Nucleo board, running FreeRTOS + micro-ROS, that
reads a BNO055 IMU over I2C and publishes live orientation data as a
`sensor_msgs/msg/Imu` message on `/imu/data`, for use by a ROS 2 host
(see the companion visualization repo below).

## Part of a larger project

This is the low-level control/sensing half of an autonomous robotized
vehicle project (obstacle avoidance, perception, trajectory planning,
25 km/h target speed). Two-board architecture:

- **STM32F401RE Nucleo** (this repo) — real-time sensor reading and
  low-level control, FreeRTOS + micro-ROS
- **Jetson Nano** — high-level perception/decision-making, ROS 2 Humble

Communication: UART/DMA serial transport → micro-ROS agent → ROS 2 topics.

**Companion repo:** [imu-visualization-ros2](https://github.com/Fattouma02/imu-visualization-ros2)
— the ROS 2 Humble workspace that subscribes to `/imu/data` and visualizes
live IMU orientation on a 3D cube in RViz2.

## Hardware

- STM32F401RE Nucleo board
- BNO055 IMU (I2C, NDOF fusion mode, quaternion output), address `0x28`
  (write byte `0x50`), CHIP_ID `0xA0`

## What this firmware does

- Initializes the BNO055 over I2C (≥700ms power-on stabilization delay
  before first read; NDOF mode via `OPR_MODE = 0x0C`)
- Reads fused quaternion orientation, angular velocity, and linear
  acceleration
- Publishes `sensor_msgs/msg/Imu` on `/imu/data` at ~20 Hz via micro-ROS,
  `frame_id = imu_link`
- Transport: UART2 + DMA to the micro-ROS agent running on the ROS 2 host

## Build

Built with STM32CubeIDE. All custom code lives inside `USER CODE BEGIN/END`
marker pairs so CubeMX regeneration doesn't wipe it.

**Important CubeMX setting:** "Generate peripheral initialization as a pair
of '.c/.h' files" must remain enabled, or `MX_FREERTOS_Init()` gets removed
from the generated output on regeneration.

## Status

- ✅ micro-ROS + BNO055 pipeline confirmed working end-to-end:
  `/imu/data` publishing at ~20 Hz with valid quaternion and linear
  acceleration (`ros2 topic hz` confirmed ~19.68 Hz stable)
- ✅ Validated live in RViz2 via the companion visualization repo — rotating
  the physical board rotates a TF-driven cube in real time

**Full pipeline running across 4 terminals** — micro-ROS agent, tf2_broadcaster,
robot_state_publisher, and RViz2:

![Four-terminal pipeline](docs/images/four_terminal_pipeline.png)

**Live result in RViz2** — orange cube with TF axes, driven by the BNO055
orientation quaternion. Physically rotating the board rotates the cube in
real time:

![RViz2 cube visualization](docs/images/rviz2_cube_visualization.png)