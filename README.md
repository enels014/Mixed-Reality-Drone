# DroneVRController

A Unity script for controlling a VR drone using **Meta Quest controllers** and **hand tracking**.

## Features

- Controller support:
  - Left joystick → strafe left/right & move forward/back
  - Right joystick X → rotate (yaw)
  - Left trigger → ascend
  - Right trigger → descend
- Hand tracking support:
  - Palm up/down → ascend/descend
  - Palm forward/back → move forward/back
  - Wrist offset → strafe left/right
  - Pinch → rotate (yaw)
  - Fist → hover

## Installation

1. Copy `DroneController.cs` into your Unity project’s **Scripts** folder.
2. Attach the script to a **Rigidbody drone GameObject**.
3. Make sure **XR Hands** and **OpenXR Hand Tracking** are enabled in Unity.

## Usage

- Works with **Meta Quest Pro** controllers.
- Automatically uses hand tracking when hands are detected, otherwise falls back to controllers.


