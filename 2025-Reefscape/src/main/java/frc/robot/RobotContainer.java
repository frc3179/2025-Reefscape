// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Auto.DriveToPose2d;
import frc.robot.Commands.Auto.ElevatorMoveToPoint;
import frc.robot.Commands.Auto.RotateDriveToPoint;
import frc.robot.Commands.Teleop.TeleopAlgae;
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
import frc.robot.Subsystems.AlgaeInOutTakeSubsystem;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.AlgaeWristSubsystem;
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
  private final AlgaeInOutTakeSubsystem m_AlgaeInOutTakeSubsystem = new AlgaeInOutTakeSubsystem();
  private final AlgaeWristSubsystem m_AlgaeWristSubsystem = new AlgaeWristSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem(m_AlgaeInOutTakeSubsystem, m_AlgaeWristSubsystem);
  
  // Other objects
  private final DriveSpeedSettings m_DriveSpeedSettings = new DriveSpeedSettings(
    SpeedSettingsConstants.kDriveSlowModePCT,
    SpeedSettingsConstants.kDriveDefaultModePCT,
    SpeedSettingsConstants.kDriveFastModePCT
  );

  private SendableChooser<Command> autoChooser;
  private AutoSubsystem m_AutoSubsystem = new AutoSubsystem();

  private TrackingSubsystem m_TrackingSubsystem = new TrackingSubsystem();

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
  
      m_AutoSubsystem.setValues(autoChooser);
  
      //TODO: TEST
      m_TrackingSubsystem.setValues(
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
        () -> !m_driverController.getRightBumperButton(),
        () -> m_driverController.getLeftTriggerAxis() >= OIConstants.kDriveTriggerDeadband,
        () -> m_driverController.getRightTriggerAxis() >= OIConstants.kDriveTriggerDeadband,
        () -> (double)m_driverController.getPOV(),
        () -> m_driverController.getAButton()
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
        () -> m_armController.getRawButton(10) ? 0.25 : (m_armController.getRawButton(9) ? -0.25 : 0.0)
        )
    );

    m_branchCoralOuttake.setDefaultCommand(
      new TeleopBranchCoralOuttake(
        m_branchCoralOuttake,
        () -> m_armController.getRawButton(1) ? 1.0 : (m_armController.getRawButton(2) ? -1.0 : 0.0)
      )
    );

    // m_TrackingSubsystem.setDefaultCommand(
    //   new TeleopTracking(
    //     m_TrackingSubsystem,
    //     TrackingConstants.kReefLimelightName,
    //     TrackingConstants.kReefLimelightName,
    //     TrackingConstants.kReefLimelightName,
    //     () -> m_robotDrive.getGryoAngle(),
    //     () -> m_robotDrive.getWheelPosition(),
    //     () -> m_robotDrive.getGryoRate(),
    //     TrackingConstants.visionMeasurementStdDevs1,
    //     TrackingConstants.visionMeasurementStdDevs2,
    //     TrackingConstants.visionMeasurementStdDevs3
    //   )
    // );

    // m_FieldSubsystem.setDefaultCommand(
    //   new RunCommand(
    //     () -> m_FieldSubsystem.updateRobotPose(m_AutoSubsystem.getInitPose()),
    //     m_FieldSubsystem
    //   )
    // );
    

    m_algaeSubsystem.setDefaultCommand(
      new TeleopAlgae(
        m_algaeSubsystem,
        () -> m_armController.getRawButton(5) ? 1.0 : (m_armController.getRawButton(3) ? -1.0 : 0.0), //In Out Take Speed
        () -> m_armController.getRawButton(7) ? 0.3 : (m_armController.getRawButton(8) ? -0.3 : 0.0) //Wrist speed
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

    new JoystickButton(m_driverController, edu.wpi.first.wpilibj.XboxController.Button.kB.value)
        .whileTrue(
          new DriveToPose2d(
            m_TrackingSubsystem,
            m_robotDrive,
            m_AutoSubsystem.getInitPose(),
            () -> m_driverController.getAButtonReleased()
          )
        );
    new JoystickButton(m_armController, 11)
        .whileTrue(
          new ElevatorMoveToPoint(
            m_elevator,
            TrackingConstants.kElevatorEncoderL2Position,
            () -> m_elevator.getEncoder(),
            TrackingConstants.kElevatorEncoderOffset,
            () -> m_armController.getRawButtonReleased(11),
            TrackingConstants.kElevatorL2P,
            TrackingConstants.kElevatorL2I,
            TrackingConstants.kElevatorL2D
          )
        );

    new JoystickButton(m_armController, 12)
        .whileTrue(
          new ElevatorMoveToPoint(
            m_elevator,
            TrackingConstants.kElevatorEncoderL3Position,
            () -> m_elevator.getEncoder(),
            TrackingConstants.kElevatorEncoderOffset,
            () -> m_armController.getRawButtonReleased(12),
            TrackingConstants.kElevatorL3P,
            TrackingConstants.kElevatorL3I,
            TrackingConstants.kElevatorL3D
          )
        );
    
    new JoystickButton(m_armController, 6)
        .whileTrue(
          new ElevatorMoveToPoint(
            m_elevator,
            TrackingConstants.kElevatorEncoderL4Position,
            () -> m_elevator.getEncoder(),
            TrackingConstants.kElevatorEncoderOffset,
            () -> m_armController.getRawButtonReleased(6),
            TrackingConstants.kElevatorL4P,
            TrackingConstants.kElevatorL4I,
            TrackingConstants.kElevatorL4D
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