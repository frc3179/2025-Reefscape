package frc.robot.Subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrackingSubsystem extends SubsystemBase {
    private SwerveDriveKinematics kinematics;
    private Rotation2d gyroAngle;
    private SwerveModulePosition[] wheelPositions;
    private Pose2d initialPoseMeters;
    private Matrix<N3, N1> stateStdDevs;
    private Matrix<N3, N1> visionMeasurementStdDevs;

    public static Odometry odometry;
    public static PoseEstimator poseEstimator;
    
    public TrackingSubsystem(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] wheelPositions,
        Pose2d initialPoseMeters,
        Matrix<N3,N1> stateStdDevs,
        Matrix<N3,N1> visionMeasurementStdDevs
    ) {
        this.kinematics = kinematics;
        this.gyroAngle = gyroAngle;
        this.wheelPositions = wheelPositions;
        this.initialPoseMeters = initialPoseMeters;
        this.stateStdDevs = stateStdDevs;
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;

        this.odometry = new Odometry(this.kinematics, this.gyroAngle, this.wheelPositions, this.initialPoseMeters);
        this.poseEstimator = new PoseEstimator(this.kinematics, this.odometry, this.stateStdDevs, this.visionMeasurementStdDevs);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
