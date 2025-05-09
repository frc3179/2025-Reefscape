// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  //public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final Pigeon2 m_gyro = new Pigeon2(50);


  StructPublisher<Pose2d> goalpublisher = NetworkTableInstance.getDefault()
  .getStructTopic("goalPose", Pose2d.struct).publish();










  public static final Lock odometryLock = new ReentrantLock();
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
      
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, rawGyroRotation, lastModulePositions, new Pose2d());

   /** Returns the current odometry pose. */
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getEstimatedRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void resetEstimatedPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getWheelPosition(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addEstimatedVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }




  
  public Command followPathToPose(Pose2d targetPose, Supplier<Boolean> flipPath) {
    if (flipPath.get()) {
      targetPose = FlippingUtil.flipFieldPose(targetPose);
    }
    goalpublisher.set(targetPose);


    return AutoBuilder.pathfindToPose(
      new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getRotation()),
      PathConstraints.unlimitedConstraints(11),
      0.0
    );
  }


  public Command aprilTagToFollowPathToPose(int aprilTagId, Supplier<Boolean> flipPath, boolean leftSide) {
    aprilTagId = Poses.redSideAprilTagIdToBlueSideId(aprilTagId);
    Pose2d[] possiblePoses = Poses.aprilTagIdToPoseOptions(aprilTagId);

    if (leftSide) {
        return this.followPathToPose(possiblePoses[0], flipPath);
    } else {
        return this.followPathToPose(possiblePoses[1], flipPath);
    }
  }







  
  
    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  
    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
      // Load the RobotConfig from the GUI settings. You should probably
      // store this in your Constants file
      RobotConfig config;
      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        config = null;
        e.printStackTrace();
      }
  
      // Configure AutoBuilder last
      AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(30.0, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
    }
  
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("MyPose", Pose2d.struct).publish();
    @Override
    public void periodic() {
      // Stop moving when disabled
      if (DriverStation.isDisabled()) {
        drive(0.0, 0.0, 0.0, false, false);
      }

      // Update odometry
      // Update the odometry in the periodic block
      m_odometry.update(
          Rotation2d.fromDegrees(-m_gyro.getAngle()),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
      });


        // Update gyro angle
        rawGyroRotation = m_gyro.getRotation2d();

        // Apply update
        poseEstimator.update(rawGyroRotation, getWheelPosition());    




        // put pose to advantage scope
        publisher.set(getEstimatedPose());
    }
  
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }
  
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
      m_odometry.resetPosition(
          Rotation2d.fromDegrees(-m_gyro.getAngle()),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          },
          pose);
    }
  
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean gyroReset) {
      if (gyroReset) {
        m_gyro.setYaw(180);
      }

      // Convert the commanded speeds into the correct units for the drivetrain
      double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
  
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                  Rotation2d.fromDegrees(-m_gyro.getAngle(/*shouldFlipPath() ? -m_gyro.getAngle() + 180 : -m_gyro.getAngle()*/)))
              : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }
  
    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
      m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }
  
    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(
          desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(desiredStates[0]);
      m_frontRight.setDesiredState(desiredStates[1]);
      m_rearLeft.setDesiredState(desiredStates[2]);
      m_rearRight.setDesiredState(desiredStates[3]);
    }
  
    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
      m_frontLeft.resetEncoders();
      m_rearLeft.resetEncoders();
      m_frontRight.resetEncoders();
      m_rearRight.resetEncoders();
    }
  
    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
      m_gyro.reset();
    }
  
    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
      return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
    }
  
    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
      return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
  
  
    //PathPlanner Made these to make auto and field localization work
  
    public ChassisSpeeds getRobotRelativeSpeeds() {
      return DriveConstants.kDriveKinematics.toChassisSpeeds(
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_rearLeft.getState(),
          m_rearRight.getState()
        );
    }
  
    public void driveRobotRelative(ChassisSpeeds speeds) {
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
  
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      setModuleStates(swerveModuleStates);
    }
  
    public boolean shouldFlipPath() {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      var alliance = DriverStation.getAlliance();
      SmartDashboard.putString("Alliance Color", "null");
      if (alliance.isPresent()) {
        SmartDashboard.putString("Alliance Color", alliance.get().toString());
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    }
  
    //Made for tracking
    public double getGryoRate() {
      return m_gyro.getRate();
    }

    public void setGryoAngle(double angle) {
      m_gyro.setYaw(angle);
    }

    public Rotation2d getGryoAngle() {
      return m_gyro.getRotation2d();
    }

    public SwerveModulePosition[] getWheelPosition() {
      return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
      };
    }
}