This is the initial codebase for the Bearbots 2026 Rebuilt robot.

The starting point was the rapid react command bot sample, to introduce the Command based robot with Epilog logging.
We rewrote the drive subsystem to use the 2026 Rev SparkMax classes.
We have employed CoPilot to generate code to implement use of the SparkMax internal PID for wheel speed, and to add the WpiLib kinematics and odometry.
