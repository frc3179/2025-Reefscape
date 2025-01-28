package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.TrackingSubsystem;

public class TeleopTracking extends Command {
    private TrackingSubsystem m_TrackingSubsystem;

    String limelightName1;
    String limelightName2;
    String limelightName3;
    Supplier<Rotation2d> gryoAngle;
    Supplier<SwerveModulePosition[]> wheelPositions;
    Supplier<Double> gryoRate;

    boolean doRejectUpdate1;
    boolean doRejectUpdate2;
    boolean doRejectUpdate3;

    public TeleopTracking(
        TrackingSubsystem m_TrackingSubsystem,
        String limelightName1,
        String limelightName2,
        String limelightName3,
        Supplier<Rotation2d> gryoAngle,
        Supplier<SwerveModulePosition[]> wheelPositions,
        Supplier<Double> gryoRate
        ) {

            this.m_TrackingSubsystem = m_TrackingSubsystem;

            this.limelightName1 = limelightName1;
            this.limelightName2 = limelightName2;
            this.limelightName3 = limelightName3;
            this.gryoAngle = gryoAngle;
            this.wheelPositions = wheelPositions;
            this.gryoRate = gryoRate;

            addRequirements(m_TrackingSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        LimelightHelpers.SetRobotOrientation(
            limelightName1,
            m_TrackingSubsystem.poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
            0,
            0,
            0,
            0,
            0
        );

        LimelightHelpers.PoseEstimate limelight1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName1);
        if(Math.abs(gryoRate.get()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate1 = true;
        }
        if(limelight1.tagCount == 0)
        {
            doRejectUpdate1 = true;
        }
        if(!doRejectUpdate1)
        {
            m_TrackingSubsystem.poseEstimator.addVisionMeasurement(
                limelight1.pose,
                limelight1.timestampSeconds
            );
        }

        LimelightHelpers.PoseEstimate limelight2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName2);
        if(Math.abs(gryoRate.get()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate2 = true;
        }
        if(limelight2.tagCount == 0)
        {
            doRejectUpdate2 = true;
        }
        if(!doRejectUpdate2)
        {
            m_TrackingSubsystem.poseEstimator.addVisionMeasurement(
                limelight1.pose,
                limelight1.timestampSeconds
            );
        }

        LimelightHelpers.PoseEstimate limelight3 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName3);
        if(Math.abs(gryoRate.get()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate3 = true;
        }
        if(limelight3.tagCount == 0)
        {
            doRejectUpdate3 = true;
        }
        if(!doRejectUpdate3)
        {
            m_TrackingSubsystem.poseEstimator.addVisionMeasurement(
                limelight1.pose,
                limelight1.timestampSeconds
            );
        }



        m_TrackingSubsystem.poseEstimator.update(gryoAngle.get(), wheelPositions.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
