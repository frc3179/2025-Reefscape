
<div align=center>![Logo](https://github.com/frc3179/2025-Reefscape/blob/main/3179.png.png)</div>


# 2025-Reefscape

This is team 3179's code for the 2025 FRC season. This repo is a place for our team to organize our code and to provide inspiration, ideas, and solutions for other teams.



## File Navigation
```
📦robot
 ┃
 ┣ 📂Commands - Folder to hold all the commands we will use.
 ┃ ┃
 ┃ ┣ 📂Auto - Sub folder that holds non joystick commands.
 ┃ ┃ ┃
 ┃ ┃ ┣ 📜DriveDriveToPoint.java - This is a general command that drives to robot forward and
 ┃ ┃ ┃                             backward to a point, which is given through parameters.
 ┃ ┃ ┃
 ┃ ┃ ┣ 📜ElevatorMoveToPoint.java - This is a general command that moves the elevator to a given,
 ┃ ┃ ┃                               point, which is given through parameters.
 ┃ ┃ ┃
 ┃ ┃ ┣ 📜FullDriveToPoint.java - This command implements the DriveDriveToPoint, RotateDriveToPoint,
 ┃ ┃ ┃                            and StrafeDriveToPoint. It also runs them at the same time.
 ┃ ┃ ┃
 ┃ ┃ ┣ 📜RotateDriveToPoint.java - This is a general command that rotates the robot to a given point,
 ┃ ┃ ┃                              which given through parameters.
 ┃ ┃ ┃
 ┃ ┃ ┗ 📜StrafeDriveToPoint.java - This is a general command that strafes the robot to a given point, 
 ┃ ┃							       which given through parameters.
 ┃ ┃
 ┃ ┗ 📂Teleop - Sub folder that holds all the default Teleop commands.
 ┃   ┃
 ┃   ┣ 📜TeleopDrive.java - File that holds the default command for driving.
 ┃   ┃
 ┃   ┗ 📜TeleopElevator.java - File for the command for moving the elevator.
 ┃  
 ┣ 📂SpeedSettings - Folder that contains all speed settings and math for all the different subsystems.
 ┃ ┃
 ┃ ┗ 📜DriveSpeedSettings.java - File that has the drive speed settings and math.
 ┃
 ┣ 📂Subsystems - Folder that holds all the subsystems.
 ┃ ┃
 ┃ ┣ 📜DriveSubsystem.java - File that uses the MAXSwerveModule Subsystem to implement the drivetrain.
 ┃ ┃
 ┃ ┣ 📜ElevatorSubsystem.java - File that makes the subsystem for the elevator.
 ┃ ┃
 ┃ ┗ 📜MAXSwerveModule.java - File that uses base moters to make a subsystem of a single swerve module.
 ┃
 ┣ 📜Configs.java - File that creates the configurations for all the REV devices. New as the update of 
 ┃		            the REV API for 2025.
 ┃
 ┣ 📜Constants.java - File that has all the constants for ports, deadbands, and limits.
 ┃
 ┣ 📜Main.java - We don't touch this file. It handles having an entry point for the binary created.
 ┃
 ┣ 📜Robot.java - Has all the places to run non-command based code to run.
 ┃
 ┗ 📜RobotContainer.java - File that makes instances of all the subsystems and joystick and provides 
			   all the commands with joystick input or buttons to start. This file also sets 
			   the auto command that we will run for that match.
```

## Contributing

Contributions are always welcome!

Create a new issue or fork for contributions and updates.


## License

Please view `LICENSE` for full information.


## Authors

- [@Hunter2718](https://github.com/Hunter2718)

