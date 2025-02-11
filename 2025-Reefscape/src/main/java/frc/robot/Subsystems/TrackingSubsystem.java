package frc.robot.Subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrackingSubsystem extends SubsystemBase {
    private SwerveDriveKinematics kinematics;
    private Rotation2d gyroAngle;
    private SwerveModulePosition[] wheelPositions;
    private Pose2d initialPoseMeters;

    public SwerveDrivePoseEstimator poseEstimator;
    
    public TrackingSubsystem() {}

    public void setValues(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] wheelPositions,
        Pose2d initialPoseMeters
    ) {
        this.kinematics = kinematics;
        this.gyroAngle = gyroAngle;
        this.wheelPositions = wheelPositions;
        this.initialPoseMeters = initialPoseMeters;

        this.poseEstimator = new SwerveDrivePoseEstimator(this.kinematics, this.gyroAngle, this.wheelPositions, this.initialPoseMeters);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
