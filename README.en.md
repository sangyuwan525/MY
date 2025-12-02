英文翻译如下：

# R1_chassis

## Introduction  
R1 chassis code for the 2026 XJRC electronics team. This season includes many updates, such as using Gitee for code management and using FreeRTOS to decouple BSP, drivers, and task functions. My skills are limited, so feedback is welcome.

## Software Architecture  
Overview of the software architecture. The code is mainly divided into three parts:
- BSP — Hardware Abstraction Layer, contains drivers for STM32 peripherals.
- DRIVER — Peripheral driver layer, contains drivers for sensors, motors, and other devices.
- Task — Application layer, contains implementations of various functional modules.

### Git Usage Guidelines  
When you first get the repository, if anything is unclear you can try writing something at the bottom of the README; after confirming functionality, proceed with formal development.

Every member with access should clone the repository and create a new local branch for development. You can push to Gitee remotely while developing, and remember to push changes to the remote repository.

After development and verifying the feature works, create a pull request on the Gitee website to merge into the main branch, wait for review by mhr, and then merge into master.

Note: Do not develop directly on the master branch to avoid conflicts.

Whenever others have pushed to master, switch to the local master branch and pull the latest changes, then switch back to your feature branch and rebase onto the updated master to ensure your branch is based on the latest master and to avoid conflicts.

When opening a pull request, please clearly describe the changes to help reviewers.

### First Update
1. Remote controller logic updated: in Task a circular buffer was added; Driver layer updated remote data processing. Current logic stores received data into a queue in the UART idle interrupt and parses/handles it in the Task layer.
2. Motor control logic updated: DJI motor CAN message sending and data updates are placed in Tasks; BSP receives CAN messages in interrupts and stores them in a queue.
3. Runtime logic updated: `Task_chassis` is only responsible for scheduling functional modules; specific functions are not finished yet.

### Second Update
1. DJI motor organization updated: motors are managed uniformly via a registry to simplify adding and removing motors. Current tests show one CAN port supports up to 6 motors simultaneously.

### Notes
- cyx joined the mhr repository.
- Why can't I submit after changing my account password?
- Tried pushing directly from CLion.
- For Gitee pushes, the username is your email address (the email used to request access). The password is your Gitee login password.