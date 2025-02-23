// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.Teleop.TeleopDrive;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpeedSettingsConstants;
import frc.robot.SpeedSettings.DriveSpeedSettings;
import frc.robot.Subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Other objects
  private final DriveSpeedSettings m_DriveSpeedSettings = new DriveSpeedSettings(
    SpeedSettingsConstants.kDriveSlowModePCT,
    SpeedSettingsConstants.kDriveDefaultModePCT,
    SpeedSettingsConstants.kDriveDefaultModePCT
    );

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure Auto Bindings
    configureAutoBindings();

    // Configure Teleop Default Bindings
    configureDefaultBindings();

    // Configure the Teleop button bindings
    configureButtonBindings();
  }

  private void configureAutoBindings() {

  }

  public void configureDefaultBindings() {

    m_robotDrive.setDefaultCommand(
      new TeleopDrive(
        m_robotDrive,
        m_DriveSpeedSettings,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveStickDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveStickDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveStickDeadband),
        () -> m_driverController.getLeftBumperButton(),
        () -> m_driverController.getLeftTriggerAxis() >= OIConstants.kDriveTriggerDeadband,
        () -> m_driverController.getRightTriggerAxis() >= OIConstants.kDriveTriggerDeadband
        )
    );

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, edu.wpi.first.wpilibj.XboxController.Button.kX.value)
        .whileTrue(
          new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive
          )
        );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}