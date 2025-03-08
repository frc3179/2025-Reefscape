/**
 * The `FieldSubsystem` class represents a subsystem for updating and displaying the pose of a robot on
 * the field in a Java FRC robot project.
 */
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

    /**
     * Updates the pose of the robot on the field.
     *
     * @param robotPose The new pose of the robot.
     */
    public void updateRobotPose(Pose2d robotPose) {
        m_field.setRobotPose(robotPose);
    }
}
