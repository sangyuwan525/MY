# R1_chassis

## Introduction
R1 chassis code for the 2026 XJRC electronic control team. This season includes many updates, such as using Gitee for source management and using FreeRTOS to decouple BSP, drivers, and task functions. My skill is limited, so feedback and corrections are welcome.

## Software Architecture
Overview of the software architecture.
The code is mainly divided into three sections:

- BSP — Hardware Abstraction Layer, containing peripheral drivers for the STM32.
- DRIVER — Peripheral driver layer, containing drivers for sensors, motors, and other peripherals.
- Task — Application layer, containing implementations of various functional modules.

### First Update
Main changes include:

1. Remote control logic updates: On the Task side, a circular buffer was added. The Driver layer updated remote control data processing. Current logic stores received data into a queue inside the UART idle interrupt, and parsing/processing is done in the Task layer.
2. Motor control logic updates: For DJI series motors, CAN message sending and data updates are handled in Tasks. The BSP layer receives CAN messages in interrupts and stores them into a queue.
3. Runtime logic updates: `Task_chassis` now only handles scheduling of modules; specific functions are not fully implemented yet.

### Notes
- User `cyx` has joined the `mhr` repository.
- If you changed your account password and can no longer commit: for Gitee commits, the username is your email address (I used the request email). The password is your Gitee login password.
