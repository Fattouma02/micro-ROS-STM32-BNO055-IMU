# Troubleshooting

Problems encountered bringing up micro-ROS on the Nucleo-F401RE, in the order they were diagnosed, with root cause and fix. Each entry is marked with how the cause was confirmed.

## 1. `librmw_microxrcedds.so: cannot open shared object file`

**Symptom:** any client-side binary (e.g. `ros2 run micro_ros_demos_rclc ping_pong`) fails immediately at launch.

**Cause (confirmed via `find`):** the micro-ROS agent and the client-side RMW implementation (`rmw_microxrcedds`) are two separate build products. Building the agent does not build the client library.

**Fix:**
```bash
ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup build_firmware.sh
```

## 2. Heap corruption running the host-side demo binary

**Symptom:** `realloc(): invalid old size`, process aborts.

**Cause:** micro-ROS's static/pool allocator is built for a microcontroller's small heap and conflicts with glibc's allocator on a host machine.

**Fix:** don't run the host-side demo binary once real firmware is on the board - test with `ros2 topic pub` / `ros2 topic echo` against the board instead.

## 3. Topic active in the agent log but invisible to `ros2 topic list` / `echo`

**Symptom:** the agent's `-v6` log shows `create_publisher`/`create_subscriber` and repeating `DataWriter::write` calls, but the host CLI shows nothing, no error.

**Cause (confirmed - `echo $ROS_DOMAIN_ID` returned empty in the affected terminal):** `ROS_DOMAIN_ID` mismatch between the agent's terminal and the CLI terminal. DDS participants on different domains never discover each other, silently.

**Fix:** export the same domain ID in every terminal:
```bash
export ROS_DOMAIN_ID=0
```
Add it to `~/.bashrc` so every new terminal inherits it.

## 4. Stale `ros2` CLI daemon after an environment change

**Symptom:** same as #3, persists even after fixing `ROS_DOMAIN_ID` in the current shell.

**Cause:** the daemon caches graph state and inherits its environment from whichever process first launched it, not the current shell.

**Fix:**
```bash
ros2 daemon stop
ros2 daemon start
```
Run this after any environment variable change, before trusting `ros2 topic list` again.

## 5. `ROS_LOCALHOST_ONLY=1` left in `~/.bashrc` from an unrelated past project

**Status: cause not independently isolated.** Found set in `.bashrc` from roughly a year earlier and removed; it may have contributed to a discovery-scoping issue similar to #3, but was not confirmed as the sole or specific cause of any one failure in this project - it was removed alongside other environment fixes in the same pass. Recorded here so it isn't rediscovered as "new" later.

**Fix applied:**
```bash
unset ROS_LOCALHOST_ONLY
```
and the corresponding `export` line removed from `~/.bashrc` permanently.

**Takeaway:** when a topic mismatch looks identical to a domain ID problem but persists after `ROS_DOMAIN_ID` is confirmed correct, check `~/.bashrc` for stale exports from unrelated past projects, not just the current shell's live environment.

## 6. Publishing to the wrong topic direction (e.g. `ros2 topic pub /pong ...`)

**Symptom:** no error, but the expected callback never fires and the LED stays off.

**Cause (confirmed from command history):** publishing manually to a topic the board itself publishes to creates a second, competing publisher instead of feeding the board's subscriber. `/ping` and `/pong` are independent topics - only the firmware's callback logic bridges one to the other.

**Fix:** check which side the board is on before publishing:
```bash
ros2 topic info /<topic> --verbose
```
`Node name: stm32_pingpong_node` + `Endpoint type: PUBLISHER` -> the board writes there, only `echo`/`info` it. `Endpoint type: SUBSCRIPTION` -> that's the one to `pub` to.

## 7. Firmware refactor: allocator declared in the wrong scope

**Symptom:** after splitting `MicroROSTask()` into helper functions, topics still register correctly in `ros2 topic list`, but the ping callback never fires - no compile error, no runtime crash.

**Cause (confirmed by code review, fixed and retested):** `rclc_support_init()` stores the allocator it's given **by pointer**, not by value. The allocator had been moved into a short-lived setup helper function; once that function returned, `support`/`executor` were left holding a dangling pointer, silently corrupting the executor's state once the infinite spin loop began.

**Fix:** declare `rcl_allocator_t allocator` directly inside `MicroROSTask()` (which never returns), and pass its address into any helper function - never let an allocator, executor, or support object be initialized against a pointer whose backing variable goes out of scope.

## 8. `region 'RAM' overflowed` at link time after a refactor

**Symptom:** build fails with `.bss will not fit in region 'RAM'`.

**Cause (confirmed from build log):** a static FreeRTOS task buffer was resized from `128` to `3000` **words** (`uint32_t[3000]` = 12,000 bytes) for a task that only calls `osDelay()` in an infinite loop. Static buffers reserve their full size in `.bss` for the program's entire lifetime regardless of actual usage.

**Fix:** size static task stacks to what the task actually needs; revert oversized idle-task buffers.

---

## Standing checklist before reporting a new issue

1. Was the board reset *after* the current agent session started?
2. Does `echo $ROS_DOMAIN_ID` match in every terminal involved, and is `ROS_LOCALHOST_ONLY` unset in all of them (check `~/.bashrc` too, not just the live shell)?
3. Has `ros2 daemon stop && ros2 daemon start` been run since the last environment change?
