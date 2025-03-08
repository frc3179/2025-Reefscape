/**
 * The `AutoSubsystem` class in the `frc.robot.Subsystems` package manages autonomous commands and
 * poses for a robot.
 */
package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSubsystem {
    SendableChooser<Command> autoChooser;
    private Pose2d startPose;

    public AutoSubsystem() {
        startPose = new Pose2d(0, 0, new Rotation2d(0));
    }

    /**
     * Sets the values of the SendableChooser for autonomous commands.
     *
     * @param autoChooser The SendableChooser containing autonomous commands
     */
    public void setValues(SendableChooser<Command> autoChooser) {
        this.autoChooser = autoChooser;
    }

    /**
     * Retrieves the current pose from the AutoBuilder and returns the selected auto command.
     *
     * @return The auto command selected from the autoChooser
     */
    public Command getAuto() {
        startPose = AutoBuilder.getCurrentPose();
        return autoChooser.getSelected();
    }

    //TODO: TEST THIS
    /**
     * Retrieves the initial pose of an object.
     *
     * @return The initial pose (position and orientation) of the object.
     */
    public Pose2d getInitPose() {
        return startPose;
    }

}
