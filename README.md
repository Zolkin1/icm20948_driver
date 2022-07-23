ICM 20948 IMU Device Driver

There have been a number of odd issues:
- First time I use the device each day, the probe-run feature doesn't work. Normally this can be fixed by connecting to it
with JLinkExe over SWD (not JTAG) and manually erasing the flash memory. Then probe-run should work fine.
- One time (so far) I got an error saying that the RTT block could not be found. I got around this issue by making the
idle loop not wfi (wake from interrupt). This appears to have solved the issue, although I am not entirely convinced this was the cause.