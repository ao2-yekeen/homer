# ACT pick-and-place experiment

Homer's SO-ARM101 was trained to pick up a carton and place it in a container
using ACT imitation learning. A Raspberry Pi recorded a wrist camera, a Logitech
overview camera and six joint positions at 10 Hz through
`act/scripts/start-demos.sh`. Training used 56 full demonstrations and later
18 hand-guided corrections, where a person finished an unsuccessful policy
attempt. The policy learned to predict sequences of joint positions from the
two camera views and the current arm position.

`act/scripts/demo.sh` ran inference on the PC and sent movements to the Pi,
executing 50 predicted actions before taking another observation. The arm
approached the carton, but reliable pickup and placement were not achieved.
Trying only five actions between observations left it stuck near the starting
pose. Adding the corrections did not resolve the task.

The investigation found pauses that taught conflicting movements from almost
identical observations, prediction errors during important parts of the task,
and a fixed stopping point that did not check whether placement had happened.
The exact cause of the failed grasp remains unresolved; this experiment did
not produce a working autonomous pick-and-place policy.

The next attempt will use a camera to track a person's hand movements and
teleoperate the arm while recording demonstrations. The aim is to avoid buying
a separate leader arm and keep the cost down. This approach has not yet been
tested.
