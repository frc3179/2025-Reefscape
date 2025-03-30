// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    FollowPathCommand.warmupCommand().schedule();
    CanBridge.runTCP();
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //SmartDashboard.putNumber("Goal Elevator Pose", TrackingConstants.kElevatorEncoderL3Position);
  }

  @Override
  public void disabledInit() {
    m_robotContainer.m_robotDrive.drive(0, 0, 0, false, false);
    m_robotContainer.m_elevator.setElevatorSpeed(0);
    m_robotContainer.m_algaeSubsystem.move(0, 0);
    m_robotContainer.m_branchCoralOuttake.setSpeed(0.0);
    m_robotContainer.m_ClimbingSubsystem.setSpeed(0);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }


    if (m_robotContainer.m_lightSubsystem.isBlueAlliance()) {
      m_robotContainer.m_robotDrive.setGryoAngle(180);
    } else {
      m_robotContainer.m_robotDrive.setGryoAngle(0);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
