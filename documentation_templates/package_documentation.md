### Rename to README.md when placed in package (Delete this sentence)

# package_name

## Description

Pargraph on how this package fits into the overall architecture. What its nodes do and what is their general (common) purpose. Why do nodes in this package belong in this package?

## Goal 

What is the goal of the code and why it was done the way it was. This will help with tech report.

## Build Instructions 

* Anything extra steps for buidling this package and running the nodes (ex. install)
  * `sudo apt-get install ...`

## Nodes

### node_name

* Node Inforation
 * What exactly deos this Node do 
 * Where does it run
 * What are its dependencies 
 * Any other extra information that should be known about this node (dynamic reconfigs, params, etc.)

example ---------------------------------------------------------------------------------------------

file: hw_thruster_controller_interface.py

Node name:
* hw_thruster_controller_interface

Topics:

* `rov/cmd_horizontal_vdrive`:
  Subscribes `vector_drive/thrusterPercents` gives the thruster setting from -1000 to 1000 for thrusters T1,2,3,4.
  * `rov/cmd_vertical_vdrive`:
  Subscribes `vector_drive/thrusterPercents`gives the thruster setting from -1000 to 1000 for thrusters T5,6,7,8.
* `topic_name`:
  Publishes `message_type` info.

Services:
* `service_name`: info

Parameters/Reconfigs:
*  `parameter_name`: info


### other_node_name (if applicable)

* Node Inforation
 * What exactly deos this Node do 
 * Where does it run
 * What are its dependencies 
 * Any other extra information that should be known about this node (dynamic reconfigs, params, etc.)
 

## Launch Information
 
Any details about launch files and what they do goes here.
Any remapping information goes here.

## Troubleshooting

## Contributors 

* Current maintaner: 

* Contributors:
  * name (anyone who writes stuff into this package) - role (what was the work said person did)

## Helpful Resources

* Links, information, external articles that were helpful in creating anything in this package


