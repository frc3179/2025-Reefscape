// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Auto.DriveToPose2d;
import frc.robot.Commands.Auto.ElevatorMoveToPoint;
import frc.robot.Commands.Teleop.TeleopBranchCoralOuttake;
import frc.robot.Commands.Teleop.TeleopDrive;
import frc.robot.Commands.Teleop.TeleopElevator;
import frc.robot.Commands.Teleop.TeleopTracking;
import frc.robot.Commands.Teleop.TeleopTroughCoralOuttake;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpeedSettingsConstants;
import frc.robot.Constants.TrackingConstants;
import frc.robot.SpeedSettings.DriveSpeedSettings;
import frc.robot.Subsystems.AutoSubsystem;
import frc.robot.Subsystems.BranchCoralOuttakeSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.FieldSubsystem;
import frc.robot.Subsystems.TrackingSubsystem;
import frc.robot.Subsystems.TroughCoralOuttakeSubsystem;
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
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final TroughCoralOuttakeSubsystem m_troughCoralOuttake = new TroughCoralOuttakeSubsystem();
  private final BranchCoralOuttakeSubsystem m_branchCoralOuttake = new BranchCoralOuttakeSubsystem();
  private final FieldSubsystem m_FieldSubsystem = new FieldSubsystem();

  private TrackingSubsystem m_TrackingSubsystem;
  
  // Other objects
  private final DriveSpeedSettings m_DriveSpeedSettings = new DriveSpeedSettings(
    SpeedSettingsConstants.kDriveSlowModePCT,
    SpeedSettingsConstants.kDriveDefaultModePCT,
    SpeedSettingsConstants.kDriveFastModePCT
  );

  private SendableChooser<Command> autoChooser;
  private AutoSubsystem m_AutoSubsystem;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_armController = new Joystick(OIConstants.kArmControllerPort);
  
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
      //TODO: AUTO COMMANDS
  
      // Build an auto chooser. This will use Commands.none() as the default option.
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
  
      m_AutoSubsystem = new AutoSubsystem(autoChooser);
  
      m_TrackingSubsystem = new TrackingSubsystem(
        DriveConstants.kDriveKinematics,
        m_robotDrive.getGryoAngle(),
        m_robotDrive.getWheelPosition(),
        m_AutoSubsystem.getInitPose()
      );
    }

  public void configureDefaultBindings() {

    m_robotDrive.setDefaultCommand(
      new TeleopDrive(
        m_robotDrive,
        m_DriveSpeedSettings,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveStickDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveStickDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveStickDeadband),
        () -> !m_driverController.getLeftBumperButton(),
        () -> m_driverController.getLeftTriggerAxis() >= OIConstants.kDriveTriggerDeadband,
        () -> m_driverController.getRightTriggerAxis() >= OIConstants.kDriveTriggerDeadband,
        () -> (double)m_driverController.getPOV()
        )
    );

    m_elevator.setDefaultCommand(
      new TeleopElevator(
        m_elevator,
        () -> MathUtil.applyDeadband(m_armController.getY(), OIConstants.kArmControllerDeadband)
      )
    );

    m_troughCoralOuttake.setDefaultCommand(
      new TeleopTroughCoralOuttake(
        m_troughCoralOuttake,
        () -> m_armController.getRawButtonPressed(1) ? 1.0 : (m_armController.getRawButtonPressed(2) ? -1.0 : 0.0)
        )
    );

    m_branchCoralOuttake.setDefaultCommand(
      new TeleopBranchCoralOuttake(
        m_branchCoralOuttake,
        () -> m_armController.getRawButtonPressed(3) ? 1.0 : 0.0
      )
    );

    m_TrackingSubsystem.setDefaultCommand(
      new TeleopTracking(
        m_TrackingSubsystem,
        TrackingConstants.kTroughIntakeLimelightName,
        TrackingConstants.kBranchIntakeLimelightName,
        TrackingConstants.kReefLimelightName,
        () -> m_robotDrive.getGryoAngle(),
        () -> m_robotDrive.getWheelPosition(),
        () -> m_robotDrive.getGryoRate(),
        TrackingConstants.visionMeasurementStdDevs1,
        TrackingConstants.visionMeasurementStdDevs2,
        TrackingConstants.visionMeasurementStdDevs3
      )
    );

    m_FieldSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_FieldSubsystem.updateRobotPose(TrackingSubsystem.poseEstimator.getEstimatedPosition()),
        m_FieldSubsystem
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

    new JoystickButton(m_driverController, edu.wpi.first.wpilibj.XboxController.Button.kA.value)
        .whileTrue(
          new DriveToPose2d(
            m_TrackingSubsystem,
            m_robotDrive,
            m_AutoSubsystem.getInitPose(),
            () -> m_driverController.getAButtonReleased()
          )
        );
    
    new JoystickButton(m_armController, 4)
        .whileTrue(
          new ElevatorMoveToPoint(
            m_elevator,
            TrackingConstants.kElevatorEncoderIntakePosition,
            () -> m_elevator.getEncoder(),
            TrackingConstants.kElevatorEncoderOffset,
            () -> m_armController.getRawButtonReleased(4)
          )
        );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_AutoSubsystem.getAuto();
  }
}