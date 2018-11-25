# racecar_teleop

## Description

This pacakge contains the control teleop nodes for both keyboard and xbox control

## Goal

To design a easy-to-use way to control the jetson car.

## Build Instructions

* `sudo apt-get install xboxdrv`

## Nodes

### joy_teleop

file: joy_teleop.py

Topics:

* `/racecar/ackermann_cmd`:
  Publishes `AckermannDriveStamped`. Gives the control state for the car

* `joy`:
  Subscribes `Joy`. Inputs the joystick data from the xbox controller

### keyop

Allows the user to control the car using the keyboard.

Moving around:

        w

   a    s    d

anything else : stop

CTRL-C to quit

file: keyboard_teleop.py

Topics:

* `/racecar/ackermann_cmd`:
  Publishes `AckermannDriveStamped`. Gives the control state for the car

## Troubleshooting

## Contributors

* Current maintaner: Caelin Sutch

* Contributors:
  * Michael Equi - Initial Work
  * Caelin Sutch - Documentation/Edits
