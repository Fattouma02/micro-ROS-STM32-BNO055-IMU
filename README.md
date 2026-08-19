# micro-ROS STM32F401RE — ping/pong

A minimal working integration of **micro-ROS** on an **STM32 Nucleo-F401RE**, running under **FreeRTOS (CMSIS-RTOS v2)**, communicating with a **ROS 2 Humble** micro-ROS agent over a serial (UART/DMA) transport.

This is the first validated milestone of a larger project (autonomous ground vehicle: Jetson Nano for perception/planning, STM32F401RE for real-time low-level control). It exists to confirm the full toolchain — CubeMX code generation, the Docker-built `libmicroros.a`, DMA-based UART transport, and FreeRTOS task integration — before adding real sensors and motor control.

## What it does

- A dedicated FreeRTOS task (`MicroROSTask`) initializes micro-ROS over `USART2` (the Nucleo's ST-Link Virtual COM Port) using a custom DMA transport.
- Subscribes to `/ping` (`std_msgs/Int32`).
- On each message received, publishes `/pong` (`std_msgs/Int32`) = `ping value + 1`, and toggles the onboard LED (`LD2`) as a visual heartbeat.
- Runs alongside the default FreeRTOS idle task, with a static (non-heap) 16 KB stack dedicated to the micro-ROS task so it doesn't compete with micro-ROS's own internal allocations.

## Hardware

- **Board:** ST Nucleo-F401RE (STM32F401RE, Cortex-M4)
- **Transport:** USB (ST-Link Virtual COM Port) → `USART2` @ 115200 baud, 8N1
- **Indicator:** onboard LED `LD2` (PA5) — blinks on `/ping` traffic; also used for error signaling (see below)

## Software stack

- STM32CubeIDE / STM32CubeMX (HAL + FreeRTOS CMSIS-RTOS v2)
- [`micro_ros_stm32cubemx_utils`](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils) for the STM32-targeted `libmicroros.a` (built via the official micro-ROS Docker builder, ROS 2 Humble)
- ROS 2 Humble + `micro_ros_agent` running on the host PC (Ubuntu)

## Repository layout

```
Core/Src/freertos.c   -- all micro-ROS + FreeRTOS application logic
Core/Src/main.c       -- stock CubeMX-generated init (clocks, GPIO, DMA, USART2)
Core/Src/dma_transport.c, microros_allocators.c, microros_time.c,
Core/Src/custom_memory_manager.c
                      -- extra_sources from micro_ros_stm32cubemx_utils
                         (transport, FreeRTOS-based allocators, clock_gettime)
*.ioc                 -- CubeMX project configuration
micro_ros_stm32cubemx_utils/
                      -- micro-ROS STM32 build utilities (git submodule / vendored)
```

## Building

1. Open the project in STM32CubeIDE (workspace already contains `microrosProject.ioc`).
2. On first build, CubeIDE runs the pre-build Docker step to (re)generate `libmicroros.a` for this target (ROS 2 Humble, STM32F401xE). Requires Docker installed and the current user in the `docker` group.
3. Build normally (`Project → Build`).
4. Flash via ST-Link (`Run → Debug` or `Run → Run`).

## Running

On the PC (ROS 2 Humble sourced):

```bash
# Terminal 1 — micro-ROS agent, serial transport
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

# Terminal 2 — publish a ping
ros2 topic pub /ping std_msgs/msg/Int32 "{data: 1}" --once

# Terminal 3 — watch for the pong
ros2 topic echo /pong
```

Expected: each `/ping` publish produces a `/pong` message with `data = ping.data + 1`, and `LD2` blinks.

## Notes on regenerating from the `.ioc`

STM32CubeMX regenerates `main.c`/`freertos.c` on every `.ioc` change, merging only what's inside `USER CODE BEGIN/END` marker pairs. In this project's current CubeMX/CubeIDE version, the FreeRTOS template does **not** provide `ThreadAttributes` or a `1` tag — so the micro-ROS task definition lives in the `Variables` tag, and `fatal_blink()`/the `RCCHECK` macros/`ping_callback()` live at the top of the `Application` tag, rather than in the locations a generic micro-ROS/STM32 tutorial might suggest. Keep **`Project Manager → Code Generator → "Generate peripheral initialization as a pair of '.c/.h' files"`** checked — unchecking it removes `MX_FREERTOS_Init()` entirely and inlines task creation into `main.c`.

## Status / next steps

- [x] micro-ROS + FreeRTOS integration validated (this repo)
- [ ] BNO055 IMU over I2C → publish `sensor_msgs/Imu` on `/imu/data`
- [ ] Migrate stack to Jetson Nano
- [ ] Integrate Livox Mid-360 LiDAR and OAK-D Lite camera

## License

*(add your license of choice — e.g. MIT — here)*
