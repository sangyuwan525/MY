# R1_chassis

## Introduction
R1 chassis code for the 2026 XJRC electronic control team. This season includes many updates, such as using Gitee for source management and using FreeRTOS to decouple BSP, drivers, and task functions. My skills are limited; feedback and corrections are welcome.

## Software Architecture
Overview of the software architecture.
The code is mainly divided into three parts:

- BSP — Hardware Abstraction Layer, containing peripheral drivers for the STM32.
- DRIVER — Peripheral driver layer, containing drivers for sensors, motors, and other peripherals.
- Task — Application layer, containing implementations of various functional modules.

### Git usage requirements
When you first get the repository, if anything is uncertain you can try writing something at the bottom of the README to test before formal development.

Each member with access should clone the repository and create a new local branch for development. You can push to the Gitee remote as usual; remember to commit and push your changes.

After development, ensure the feature works correctly, then create a Pull Request on the Gitee website to merge into the main branch. Wait for review by `mhr`; after approval it can be merged into `master`.

Note: Do not develop directly on the `master` branch to avoid conflicts.

Whenever others have pushed to `master`, switch to `master` locally and pull the latest code first, then switch back to your development branch and perform a rebase onto `master` to ensure your branch is based on the latest `master` and to avoid conflicts.

When creating a Pull Request, please clearly write the update contents below to help reviewers.

### First update
1. Remote control logic updated: On the Task side a circular buffer was added; the Driver layer updated remote control data processing. Current logic stores received data into a queue in the UART idle interrupt, and parsing/processing is done in the Task layer.
2. Motor control logic updated: For DJI series motors, CAN message sending and data updates are handled in Tasks. The BSP layer receives CAN messages in interrupts and stores them into a queue.
3. Runtime logic updated: `Task_chassis` now only handles scheduling of functional modules; specific functions are not fully implemented yet.

### Notes
- `cyx` has joined the `mhr` repository.
- If you changed your account password and can no longer commit: for Gitee commits, the username is your email address (I used the request email). The password is your Gitee login password.
