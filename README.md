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
2. If you want my drone, then view raw in the `MixedRealityDrone.blend` file, open it in blender, save it as a file somwehere, drag it into your assets folder in unity, and then drag it into your scene
3. Attach the script to a **Rigidbody drone GameObject**.
4. Make sure **XR Hands** and **OpenXR Hand Tracking** are enabled in Unity.
5. Att

## Usage

- Works with **Meta Quest Pro** controllers.
- Automatically uses hand tracking when hands are detected, otherwise falls back to controllers.


