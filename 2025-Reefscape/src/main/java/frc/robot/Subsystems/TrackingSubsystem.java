/**
 * The `TrackingSubsystem` class in Java implements a subsystem for tracking the pose of an object
 * using a Swerve drive pose estimator.
 */
package frc.robot.Subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrackingSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    private StructPublisher<Pose2d> publisher; 
    
    public TrackingSubsystem(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] wheelPositions,
        Pose2d initialPoseMeters
    ) {

        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyroAngle,
            wheelPositions,
            initialPoseMeters
        );

        publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Robot Pose", Pose2d.struct).publish();
    }

    /**
     * Retrieves the estimated pose (position and orientation) from the pose estimator.
     *
     * @return The estimated pose (position and orientation) of the object.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        publisher.set(getEstimatedPose());
    }
}