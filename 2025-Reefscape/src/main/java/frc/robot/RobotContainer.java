// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Auto.BranchIntake;
import frc.robot.Commands.Auto.BranchIntakeNoInteruptStopMotor;
import frc.robot.Commands.Auto.BranchOuttake;
import frc.robot.Commands.Auto.ElevatorMoveToPoint;
import frc.robot.Commands.Auto.FullDriveToPoint;
import frc.robot.Commands.Auto.ThroughCoralOuttakeToPoint;
import frc.robot.Commands.Teleop.TeleopAlgae;
import frc.robot.Commands.Teleop.TeleopBranchCoralOuttake;
import frc.robot.Commands.Teleop.TeleopClimb;
import frc.robot.Commands.Teleop.TeleopDrive;
import frc.robot.Commands.Teleop.TeleopElevator;
import frc.robot.Commands.Teleop.TeleopLights;
import frc.robot.Commands.Teleop.TeleopTracking;
import frc.robot.Commands.Teleop.TeleopTroughCoralOuttake;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LightSubsystemConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpeedSettingsConstants;
import frc.robot.Constants.TrackingConstants;
import frc.robot.SpeedSettings.DriveSpeedSettings;
import frc.robot.Subsystems.AlgaeInOutTakeSubsystem;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.AlgaeWristSubsystem;
import frc.robot.Subsystems.AutoSubsystem;
import frc.robot.Subsystems.BranchCoralOuttakeSubsystem;
import frc.robot.Subsystems.ClimbingSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.LightSubsystem;
import frc.robot.Subsystems.TrackingSubsystem;
import frc.robot.Subsystems.TroughCoralOuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final AlgaeInOutTakeSubsystem m_AlgaeInOutTakeSubsystem = new AlgaeInOutTakeSubsystem();
  private final AlgaeWristSubsystem m_AlgaeWristSubsystem = new AlgaeWristSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem(m_AlgaeInOutTakeSubsystem, m_AlgaeWristSubsystem);
  private final LightSubsystem m_lightSubsystem = new LightSubsystem(LightSubsystemConstants.kBlinkinPort);
  private final ClimbingSubsystem m_ClimbingSubsystem = new ClimbingSubsystem();
  
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
  Joystick m_autoController = new Joystick(2);
  
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

      autoChooser = AutoBuilder.buildAutoChooser();
      autoChooser.setDefaultOption("Still", new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive));
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }
  
    private void configureAutoBindings() {
      //TODO: AUTO COMMANDS

      //TODO: needs offset to right
      NamedCommands.registerCommand(
        "Robot Line Up",
        new FullDriveToPoint(
            m_robotDrive,
            TrackingConstants.kReefForwardLimelightOffset, //Goal Drive
            () -> LimelightHelpers.getTY(TrackingConstants.kStillLimelightName), //Drive Current
            0.3, //Drive Error offset
            0.0, //Goal Rotate
            () -> 0.0, //Rotate Current
            0.1, //Rotate Error offset
            TrackingConstants.kRightReefStrafeLimelightOffset, //Goal Strafe
            () -> LimelightHelpers.getTY(TrackingConstants.kStillLimelightName) < -10.0 ? LimelightHelpers.getTX(TrackingConstants.kStillLimelightName) : TrackingConstants.kRightReefStrafeLimelightOffset+5, //Strafe Current
            0.3, //Stra fe Error offset
            () -> false
          ).withTimeout(1.3) 
      );

      NamedCommands.registerCommand(
        "L4 Score",
        new SequentialCommandGroup(
          new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderL4Position,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> false,
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            ),

            new BranchOuttake(
              m_branchCoralOuttake,
              () -> false
            ),

            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderIntakePosition,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> false,
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            )
        )
      );

      NamedCommands.registerCommand(
        "L3 Score",
        new SequentialCommandGroup(
            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderL3Position,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> false,
              TrackingConstants.kElevatorL3P,
              TrackingConstants.kElevatorL3I,
              TrackingConstants.kElevatorL3D
            ),

            new BranchOuttake(
              m_branchCoralOuttake,
              () -> false
            ),

            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderIntakePosition,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> false,
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            )
          )
      );

      NamedCommands.registerCommand(
        "L2 Score",
        new SequentialCommandGroup(
            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderL2Position,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> false,
              TrackingConstants.kElevatorL2P,
              TrackingConstants.kElevatorL2I,
              TrackingConstants.kElevatorL2D
            ),

            new BranchOuttake(
              m_branchCoralOuttake,
              () -> false
            ),

            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderIntakePosition,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> false,
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            )
          )
      );

      NamedCommands.registerCommand(
        "Branch Intake",
        new BranchIntakeNoInteruptStopMotor(
          m_branchCoralOuttake
        ).withTimeout(2)
      );

  
      //TODO: TEST
      m_TrackingSubsystem.setValues(
        DriveConstants.kDriveKinematics,
        m_robotDrive.getGryoAngle(),
        m_robotDrive.getWheelPosition(),
        m_AutoSubsystem.getCurrentAutoPose()
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
        () -> m_driverController.getYButton()
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

    m_TrackingSubsystem.setDefaultCommand(
      new TeleopTracking(
        m_TrackingSubsystem,
        TrackingConstants.kStillLimelightName, //first to update pose
        TrackingConstants.kStillLimelightName, //second to update pose
        TrackingConstants.kStillLimelightName,//TrackingConstants.kReefLimelightName, //third to update pose
        () -> m_robotDrive.getGryoAngle(),
        () -> m_robotDrive.getWheelPosition(),
        () -> m_robotDrive.getGryoRate(),
        TrackingConstants.visionMeasurementStdDevs1,
        TrackingConstants.visionMeasurementStdDevs2,
        TrackingConstants.visionMeasurementStdDevs3
      )
    );
    

    m_algaeSubsystem.setDefaultCommand(
      new TeleopAlgae(
        m_algaeSubsystem,
        () -> m_armController.getRawButton(3) ? 1.0 : (m_armController.getRawButton(5) ? -1.0 : 0.0), //In Out Take Speed
        () -> m_armController.getRawButton(7) ? 0.3 : (m_armController.getRawButton(8) ? -0.3 : 0.0) //Wrist speed
      )
    );


    m_lightSubsystem.setDefaultCommand(
      new TeleopLights(
        m_lightSubsystem,
        () -> m_lightSubsystem.laserCanToColor(m_branchCoralOuttake.getLaserCanMeasurments()[0], m_branchCoralOuttake.getLaserCanMeasurments()[1])
      )
    );

    m_ClimbingSubsystem.setDefaultCommand(
      new TeleopClimb(
        m_ClimbingSubsystem,
        () -> m_autoController.getRawAxis(1)
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

    new JoystickButton(m_armController, 11)
        .whileTrue(
          new ElevatorMoveToPoint(
            m_elevator,
            TrackingConstants.kElevatorEncoderL2Position,
            () -> m_elevator.getEncoder(),
            TrackingConstants.kElevatorEncoderOffset,
            () -> !m_armController.getRawButton(11),
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
            () -> !m_armController.getRawButton(12),
            TrackingConstants.kElevatorL3P,
            TrackingConstants.kElevatorL3I,
            TrackingConstants.kElevatorL3D
          )
        );
    
    // L4
    new JoystickButton(m_autoController, 6)
        .onTrue(
          new SequentialCommandGroup(
            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderL4Position,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_autoController.getRawButton(2),
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            ),

            new BranchOuttake(
              m_branchCoralOuttake,
              () -> m_autoController.getRawButton(2)
            ),

            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderIntakePosition,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_autoController.getRawButton(2),
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            )
          )
        );

    // L3
    new JoystickButton(m_autoController, 4)
        .onTrue(
          new SequentialCommandGroup(
            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderL3Position,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_autoController.getRawButton(2),
              TrackingConstants.kElevatorL3P,
              TrackingConstants.kElevatorL3I,
              TrackingConstants.kElevatorL3D
            ),

            new BranchOuttake(
              m_branchCoralOuttake,
              () -> m_autoController.getRawButton(2)
            ),

            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderIntakePosition,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_autoController.getRawButton(2),
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            )
          )
        );

    // L2
    new JoystickButton(m_autoController, 3)
        .onTrue(
          new SequentialCommandGroup(
            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderL2Position,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_autoController.getRawButton(2),
              TrackingConstants.kElevatorL2P,
              TrackingConstants.kElevatorL2I,
              TrackingConstants.kElevatorL2D
            ),

            new BranchOuttake(
              m_branchCoralOuttake,
              () -> m_autoController.getRawButton(2)
            ),

            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderIntakePosition,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_autoController.getRawButton(2),
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            )
          )
        );



    new JoystickButton(m_armController, 4)
        .whileTrue(
          new ThroughCoralOuttakeToPoint(
            m_troughCoralOuttake,
            TrackingConstants.kThroughCoralOuttakeIntakePos,
            () -> m_troughCoralOuttake.getEncoder(),
            0.1,
            () -> !m_armController.getRawButton(4)
          )
        );

    
    new JoystickButton(m_autoController, 1)
        .whileTrue(
          new BranchIntake(
            m_branchCoralOuttake,
            () -> !m_autoController.getRawButton(1)
          )
        );

    new JoystickButton(m_driverController, edu.wpi.first.wpilibj.XboxController.Button.kA.value)
        .whileTrue(
          new FullDriveToPoint(
            m_robotDrive,
            0.0, //Goal Drive
            () -> 0.0, //Drive Current
            0.3, //Drive Error offset
            0.0, //Goal Rotate
            () -> 0.0, //Rotate Current
            0.1, //Rotate Error offset
            TrackingConstants.kRightReefStrafeLimelightOffset, //Goal Strafe
            () -> LimelightHelpers.getTY(TrackingConstants.kStillLimelightName) < -10.0 ? LimelightHelpers.getTX(TrackingConstants.kStillLimelightName) : TrackingConstants.kRightReefStrafeLimelightOffset+5, //Strafe Current
            0.3, //Stra fe Error offset
            () -> !m_driverController.getRawButton(edu.wpi.first.wpilibj.XboxController.Button.kA.value)
          )
        );

    
    new JoystickButton(m_driverController, edu.wpi.first.wpilibj.XboxController.Button.kB.value)
        .whileTrue(
          m_AutoSubsystem.endPoseToCommand(
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            0.0
          )
        );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}