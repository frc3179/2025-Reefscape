package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldSubsystem extends SubsystemBase{
    private Field2d m_field;

    public FieldSubsystem() {
        this.m_field = new Field2d();
        SmartDashboard.putData("Field", this.m_field);
    }

    public void updateRobotPose(Pose2d robotPose) {
        m_field.setRobotPose(robotPose);
    }
}
