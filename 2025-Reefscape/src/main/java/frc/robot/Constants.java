// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.74;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveStickDeadband = 0.1;
    public static final double kDriveTriggerDeadband = 0.1;

    public static final int kArmControllerPort = 1;
    public static final double kArmControllerDeadband = 0.15;
  }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
        );

    }

  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class TrackingConstants {
    public static final double kStrafeDriveP = 0.005;
    public static final double kStrafeDriveI = 0.03;
    public static final double kStrafeDriveD = 0.005;
    public static final double kRightReefStrafeLimelightOffset = -16.24;
    public static final double kLeftReefStrafeLimelightOffset = 0.0;
    
    public static final double kRotateDriveP = 0.01;
    public static final double kRotateDriveI = 0.0;
    public static final double kRotateDriveD = 0.0;
    public static final double kRotGoalPose = 12.4;
    
    
    public static final double kDriveDriveP = 0.15;
    public static final double kDriveDriveI = 0.03; 
    public static final double kDriveDriveD = 0.0;
    public static final double kReefForwardLimelightOffset = 12.91;
    
    public static final double kElevatorL2P = 10.0;
    public static final double kElevatorL2I = 0.005;
    public static final double kElevatorL2D = 0.0;

    public static final double kElevatorL3P = 10.0;
    public static final double kElevatorL3I = 0.005;
    public static final double kElevatorL3D = 0.0;

    public static final double kElevatorL4P = 8.0;
    public static final double kElevatorL4I = 0.005;
    public static final double kElevatorL4D = 0.0;

    public static final double kThroughCoralOuttakeP = 0.0;
    public static final double kThroughCoralOuttakeI = 0.0;
    public static final double kThroughCoralOuttakeD = 0.0;

    public static final double kElevatorEncoderIntakePosition = 0.015;
    public static final double kElevatorEncoderL2Position = 0.22;
    public static final double kElevatorEncoderL3Position = 0.37;
    public static final double kElevatorEncoderL4Position = 0.6;
    public static final double kElevatorEncoderOffset = 0.015;

    public static final double kThroughCoralOuttakeIntakePos = 0.0;
    public static final double kThroughCoralOutakeScoringPos = 1.0;

    // x position in meters, y position in meters, and heading in radians; higher number = trust less
    public static final SimpleMatrix visionMeasurementStdDevs1Data = new SimpleMatrix(new double[] {0.7, 0.7, 0.999});
    public static final Matrix<N3,N1> visionMeasurementStdDevs1 = new Matrix<N3, N1>(visionMeasurementStdDevs1Data);

    // x position in meters, y position in meters, and heading in radians; higher number = trust less
    public static final SimpleMatrix visionMeasurementStdDevs2Data = new SimpleMatrix(new double[] {0.7, 0.7, 0.999});
    public static final Matrix<N3,N1> visionMeasurementStdDevs2 = new Matrix<N3,N1>(visionMeasurementStdDevs2Data);

    // x position in meters, y position in meters, and heading in radians; higher number = trust less
    public static final SimpleMatrix visionMeasurementStdDevs3Data = new SimpleMatrix(new double[] {0.7, 0.7, 0.999});
    public static final Matrix<N3,N1> visionMeasurementStdDevs3 = new Matrix<N3,N1>(visionMeasurementStdDevs3Data);

    public static final String kMovingLimelightName = "limelight-moving";
    public static final String kStillLimelightName = "limelight-still";
    public static final String kBranchIntakeLimelightName = "limelight-branch";
  }

  public static final class SpeedSettingsConstants {
    public static final double kDriveFastModePCT = 1.0;
    public static final double kDriveSlowModePCT = 0.25;
    public static final double kDriveDefaultModePCT = 0.5;
  }

  public static final class ElevatorConstants {
    public static final int kLeftMotorPort = 10; 
    public static final int kRightMotorPort = 11;
    public static final int kEncoderPort = 10;

    //public static final double kBottomEncoder = 0.0001;
    public static final double kBottomEncoder = -1000000000;
    //public static final double kTopEncoder = 5.4;
    public static final double kTopEncoder = 100000000;
    public static final double kMaxEncoderBeforeReset = 1.0;
    public static final double kMaxThresholdForResetPercent = 0.9;
  }

  //public static final class ThroughCoralOuttakeConstants {
  //  public static final int kThroughCoralOuttakeMotorPort = 12;
  //}

  public static final class BranchCoralOuttakeConstants {
    public static final int kBranchCoralOuttakeMotorPort = 19;

    public static final int kFrontLaserCanPort = 20;
    public static final int kBackLaserCanPort = 21;
  }

  public static final class AlgaeSubsystemConstants {
    public static final int kInOutTakeMotorPort = 23;
    public static final int kWristMotorPort = 22;
  }

  public static final class LightSubsystemConstants {
    public static final int kBlinkinPort = 0;
  }

  public static final class ClimbSubsystemConstants {
    public static final int kClimbMotorPort = 13;
  }
}