package frc.robot.Subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrackingSubsystem extends SubsystemBase {
    private SwerveDriveKinematics kinematics;
    private Rotation2d gyroAngle;
    private SwerveModulePosition[] wheelPositions;
    private Pose2d initialPoseMeters;

    public static SwerveDrivePoseEstimator poseEstimator;

    private Field2d m_field;
    
    public TrackingSubsystem(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] wheelPositions,
        Pose2d initialPoseMeters
    ) {
        this.kinematics = kinematics;
        this.gyroAngle = gyroAngle;
        this.wheelPositions = wheelPositions;
        this.initialPoseMeters = initialPoseMeters;

        TrackingSubsystem.poseEstimator = new SwerveDrivePoseEstimator(this.kinematics, this.gyroAngle, this.wheelPositions, this.initialPoseMeters);

        this.m_field = new Field2d();
        SmartDashboard.putData("Field", this.m_field);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        m_field.setRobotPose(poseEstimator.getEstimatedPosition());
    }
}
