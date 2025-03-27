package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TrackingConstants;

public class LimelightVision extends SubsystemBase{
    String limelightName;
    DriveSubsystem driveSubsystem;
    TrackingSubsystem trackingSubsystem;
    public LimelightVision(DriveSubsystem driveSubsystem, TrackingSubsystem trackingSubsystem, String limelightName){
        this.limelightName = limelightName;
        this.driveSubsystem = driveSubsystem;
        this.trackingSubsystem = trackingSubsystem;
    }

    @Override
    public void periodic(){
        //update the pose estimator with the limelight values
        boolean isReject = false;
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            trackingSubsystem.poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
            0,
            0,
            0,
            0,
            0
        );

        var l1measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (l1measurement != null) {
            SmartDashboard.getNumber("Number of tags seen by " + limelightName, l1measurement.tagCount);
            // SmartDashboard.putBoolean("Limelight3", true);

            double gryoRate = driveSubsystem.getGryoRate();
            if(Math.abs(gryoRate) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            {
                isReject = true;
            }
            if(l1measurement.tagCount == 0)
            {
                isReject = true;
            }
            if(!isReject)
            {
                // m_TrackingSubsystem.poseEstimator.setVisionMeasurementStdDevs(limelight3VisionMeasurementStdDevs);
                trackingSubsystem.poseEstimator.addVisionMeasurement(
                    l1measurement.pose,
                    l1measurement.timestampSeconds,
                    TrackingConstants.visionMeasurementStdDevs1
                );
            }
        }        
        trackingSubsystem.poseEstimator.update(driveSubsystem.getGryoAngle(), driveSubsystem.getWheelPosition());
    }

}
