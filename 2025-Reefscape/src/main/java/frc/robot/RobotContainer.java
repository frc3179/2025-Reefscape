// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.StopAll;
import frc.robot.Commands.Auto.BranchIntake;
import frc.robot.Commands.Auto.BranchIntakeNoInteruptStopMotor;
import frc.robot.Commands.Auto.BranchOuttake;
import frc.robot.Commands.Auto.ClimbToPoint;
import frc.robot.Commands.Auto.ElevatorMoveToPoint;
import frc.robot.Commands.Auto.FullDriveToPoint;
//import frc.robot.Commands.Auto.GoToPose;
import frc.robot.Commands.Teleop.TeleopAlgae;
import frc.robot.Commands.Teleop.TeleopBranchCoralOuttake;
import frc.robot.Commands.Teleop.TeleopClimb;
import frc.robot.Commands.Teleop.TeleopDrive;
import frc.robot.Commands.Teleop.TeleopElevator;
import frc.robot.Commands.Teleop.TeleopLights;
import frc.robot.Constants.LightSubsystemConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpeedSettingsConstants;
import frc.robot.Constants.TrackingConstants;
import frc.robot.SpeedSettings.DriveSpeedSettings;
import frc.robot.Subsystems.Algae.AlgaeInOutTakeSubsystem;
import frc.robot.Subsystems.Algae.AlgaeSubsystem;
import frc.robot.Subsystems.Algae.AlgaeWristSubsystem;
import frc.robot.Subsystems.Climb.ClimbingSubsystem;
import frc.robot.Subsystems.Coral.BranchCoralOuttakeSubsystem;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.Poses;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Lights.LightSubsystem;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Subsystems.Vision.VisionIOLimelight;
import frc.robot.Subsystems.Vision.VisionIOPhotonVisionTrig;
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
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final BranchCoralOuttakeSubsystem m_branchCoralOuttake = new BranchCoralOuttakeSubsystem();
  public final AlgaeInOutTakeSubsystem m_AlgaeInOutTakeSubsystem = new AlgaeInOutTakeSubsystem();
  public final AlgaeWristSubsystem m_AlgaeWristSubsystem = new AlgaeWristSubsystem();
  public final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem(m_AlgaeInOutTakeSubsystem, m_AlgaeWristSubsystem);
  public final LightSubsystem m_lightSubsystem = new LightSubsystem(LightSubsystemConstants.kBlinkinPort);
  public final ClimbingSubsystem m_ClimbingSubsystem = new ClimbingSubsystem();


  @SuppressWarnings("unused")
  private final Vision vision;

  private VisionIOPhotonVisionTrig backCamera = new VisionIOPhotonVisionTrig(
    VisionConstants.camera0Name,
    VisionConstants.robotToCamera0,
    m_robotDrive::getGryoAngle
  );

  private VisionIOPhotonVisionTrig frontCamera = new VisionIOPhotonVisionTrig(
    VisionConstants.camera1Name,
    VisionConstants.robotToCamera1,
    m_robotDrive::getGryoAngle
  );
  
  // Other objects
  public final DriveSpeedSettings m_DriveSpeedSettings = new DriveSpeedSettings(
    SpeedSettingsConstants.kDriveSlowModePCT,
    SpeedSettingsConstants.kDriveDefaultModePCT,
    SpeedSettingsConstants.kDriveFastModePCT
    );
    
  public SendableChooser<Command> autoChooser;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_armController = new Joystick(OIConstants.kArmControllerPort);
  Joystick m_autoController = new Joystick(2);
  GenericHID m_buttonBoard = new GenericHID(3);
  
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {
      

      vision = new Vision(
        m_robotDrive::addEstimatedVisionMeasurement,

        backCamera,

        frontCamera
      );

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
      NamedCommands.registerCommand(
        "Right Robot Line Up",
        new FullDriveToPoint(
            m_robotDrive,
            0,
            () -> 0.0,
            0,
            0,
            () -> 0.0,
            0,
            TrackingConstants.kRightReefStrafeLimelightOffset,
            () -> LimelightHelpers.getTX(TrackingConstants.kBasicTrackingLimelightName),
            0.3,
            () -> m_buttonBoard.getRawButton(7),
            new PIDConstants(TrackingConstants.kDriveDriveP, TrackingConstants.kDriveDriveI, TrackingConstants.kDriveDriveD),
            new PIDConstants(TrackingConstants.kStrafeDriveP, TrackingConstants.kStrafeDriveI, TrackingConstants.kDriveDriveD),
            new PIDConstants(TrackingConstants.kRotateDriveP, TrackingConstants.kRotateDriveI, TrackingConstants.kDriveDriveD)
          ).withTimeout(2) 
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



      NamedCommands.registerCommand(
        "Go To A",
        m_robotDrive.followPathToPose(Poses.kReefA, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To B",
        m_robotDrive.followPathToPose(Poses.kReefB, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To C",
        m_robotDrive.followPathToPose(Poses.kReefC, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To D",
        m_robotDrive.followPathToPose(Poses.kReefD, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To E",
        m_robotDrive.followPathToPose(Poses.kReefE, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To F",
        m_robotDrive.followPathToPose(Poses.kReefF, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To G",
        m_robotDrive.followPathToPose(Poses.kReefG, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To H",
        m_robotDrive.followPathToPose(Poses.kReefH, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To I",
        m_robotDrive.followPathToPose(Poses.kReefI, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To J",
        m_robotDrive.followPathToPose(Poses.kReefJ, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To K",
        m_robotDrive.followPathToPose(Poses.kReefK, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To L",
        m_robotDrive.followPathToPose(Poses.kReefL, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To LF",
        m_robotDrive.followPathToPose(Poses.kLeftFeeder, () -> !m_robotDrive.shouldFlipPath())
      );

      NamedCommands.registerCommand(
        "Go To RF",
        m_robotDrive.followPathToPose(Poses.kRightFeeder, () -> !m_robotDrive.shouldFlipPath())
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

    // m_troughCoralOuttake.setDefaultCommand(
    //   new TeleopTroughCoralOuttake(
    //     m_troughCoralOuttake,
    //     () -> m_armController.getRawButton(10) ? 0.25 : (m_armController.getRawButton(9) ? -0.25 : 0.0)
    //     )
    // );

    m_branchCoralOuttake.setDefaultCommand(
      new TeleopBranchCoralOuttake(
        m_branchCoralOuttake,
        () -> m_armController.getRawButton(1) ? 1.0 : (m_armController.getRawButton(2) ? -1.0 : 0.0)
      )
    );    

    // m_algaeSubsystem.setDefaultCommand(
    //   new TeleopAlgae(
    //     m_algaeSubsystem,
    //     () -> m_armController.getRawButton(3) ? 1.0 : (m_armController.getRawButton(5) ? -1.0 : 0.0), //In Out Take Speed
    //     () -> m_armController.getRawButton(7) ? 1 : (m_armController.getRawButton(8) ? -1 : 0.0) //Wrist speed
    //   )
    // );

    m_algaeSubsystem.setDefaultCommand(
      new TeleopAlgae(
        m_algaeSubsystem,
        () -> m_armController.getRawButton(3) ? 1.0 : (m_armController.getRawButton(5) ? -1.0 : 0.0), //In Out Take Speed
        () -> (m_armController.getPOV() == 0 ? 1.0 : (m_armController.getPOV() == 180 ? -1.0 : 0.0)) //Wrist speed
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
        () -> m_buttonBoard.getRawButton(3) ? 1.0 : m_buttonBoard.getRawButton(1) ? -1.0 : 0.0
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
    new JoystickButton(m_buttonBoard, 2)
        .onTrue(
          new SequentialCommandGroup(
            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderL4Position,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_buttonBoard.getRawButton(7),
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            ),

            new BranchOuttake(
              m_branchCoralOuttake,
              () -> m_buttonBoard.getRawButton(7)
            ),

            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderIntakePosition,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_buttonBoard.getRawButton(7),
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            )
          )
        );

    // L3
    new JoystickButton(m_buttonBoard, 4)
        .onTrue(
          new SequentialCommandGroup(
            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderL3Position,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_buttonBoard.getRawButton(7),
              TrackingConstants.kElevatorL3P,
              TrackingConstants.kElevatorL3I,
              TrackingConstants.kElevatorL3D
            ),

            new BranchOuttake(
              m_branchCoralOuttake,
              () -> m_buttonBoard.getRawButton(7)
            ),

            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderIntakePosition,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_buttonBoard.getRawButton(7),
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            )
          )
        );

    // L2
    new JoystickButton(m_buttonBoard, 6)
        .onTrue(
          new SequentialCommandGroup(
            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderL2Position,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_buttonBoard.getRawButton(7),
              TrackingConstants.kElevatorL2P,
              TrackingConstants.kElevatorL2I,
              TrackingConstants.kElevatorL2D
            ),

            new BranchOuttake(
              m_branchCoralOuttake,
              () -> m_buttonBoard.getRawButton(7)
            ),

            new ElevatorMoveToPoint(
              m_elevator,
              TrackingConstants.kElevatorEncoderIntakePosition,
              () -> m_elevator.getEncoder(),
              TrackingConstants.kElevatorEncoderOffset,
              () -> m_buttonBoard.getRawButton(7),
              TrackingConstants.kElevatorL4P,
              TrackingConstants.kElevatorL4I,
              TrackingConstants.kElevatorL4D
            )
          )
        );



    // new JoystickButton(m_armController, 4)
    //     .whileTrue(
    //       new ThroughCoralOuttakeToPoint(
    //         m_troughCoralOuttake,
    //         TrackingConstants.kThroughCoralOuttakeIntakePos,
    //         () -> m_troughCoralOuttake.getEncoder(),
    //         0.1,
    //         () -> !m_armController.getRawButton(4)
    //       )
    //     );

    
    new JoystickButton(m_buttonBoard, 5)
        .whileTrue(
          new BranchIntake(
            m_branchCoralOuttake,
            () -> !m_buttonBoard.getRawButton(5)
          )
        );

    //Climb out
    // new JoystickButton(m_autoController, 7)
    //     .whileTrue(
    //       new ClimbToPoint(
    //         m_ClimbingSubsystem,
    //         TrackingConstants.kClimbP,
    //         TrackingConstants.kClimbI,
    //         TrackingConstants.kClimbD,
    //         TrackingConstants.kClimbOut,
    //         0.03,
    //         () -> m_ClimbingSubsystem.getEncoder(),
    //         () -> m_buttonBoard.getRawButton(7)
    //       )
    //     );

    //Climb in
    // new JoystickButton(m_autoController, 8)
    //     .whileTrue(
    //       new ClimbToPoint(
    //         m_ClimbingSubsystem,
    //         TrackingConstants.kClimbP,
    //         TrackingConstants.kClimbI,
    //         TrackingConstants.kClimbD,
    //         TrackingConstants.kClimbZero,
    //         0.03,
    //         () -> m_ClimbingSubsystem.getEncoder(),
    //         () -> m_buttonBoard.getRawButton(7)
    //       )
    //     );

    
    new JoystickButton(m_driverController, edu.wpi.first.wpilibj.XboxController.Button.kA.value)
        .onTrue(
         m_robotDrive.aprilTagToFollowPathToPose(frontCamera.getAprilTagId(), () -> !m_robotDrive.shouldFlipPath(), false)
        );
    // new JoystickButton(m_driverController, edu.wpi.first.wpilibj.XboxController.Button.kA.value)
    //     .onTrue(
    //      m_robotDrive.followPathToPose(Poses.kLeftFeeder, () -> !m_robotDrive.shouldFlipPath())
    //     );

    // new JoystickButton(m_driverController, edu.wpi.first.wpilibj.XboxController.Button.kA.value)
    //     .onTrue(
    //       AutoBuilder.pathfindToPose(
    //         new Pose2d(
    //           16.43,
    //           0.98,
    //           new Rotation2d(Units.degreesToRadians(37.16))
    //         ),
    //         PathConstraints.unlimitedConstraints(11)
    //       )
    //     );

    // new JoystickButton(m_driverController, edu.wpi.first.wpilibj.XboxController.Button.kA.value)
    //     .whileTrue(
    //       new FullDriveToPoint(
    //         m_robotDrive,
    //         0,
    //         () -> 0.0,
    //         0,
    //         0,
    //         () -> 0.0,
    //         0,
    //         TrackingConstants.kRightReefStrafeLimelightOffset,
    //         () -> LimelightHelpers.getTX(TrackingConstants.kBasicTrackingLimelightName),
    //         0.3,
    //         () -> m_buttonBoard.getRawButton(7),
    //         new PIDConstants(TrackingConstants.kDriveDriveP, TrackingConstants.kDriveDriveI, TrackingConstants.kDriveDriveD),
    //         new PIDConstants(TrackingConstants.kStrafeDriveP, TrackingConstants.kStrafeDriveI, TrackingConstants.kDriveDriveD),
    //         new PIDConstants(TrackingConstants.kRotateDriveP, TrackingConstants.kRotateDriveI, TrackingConstants.kDriveDriveD)
    //       )
    //     );


    new JoystickButton(m_buttonBoard, 7)
        .whileTrue(
          new StopAll(
            m_robotDrive,
            m_elevator,
            m_algaeSubsystem,
            m_branchCoralOuttake,
            m_ClimbingSubsystem
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