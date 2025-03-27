/**
 * The `AutoSubsystem` class in the `frc.robot.Subsystems` package manages autonomous commands and
 * poses for a robot.
 */
package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSubsystem {
    SendableChooser<Command> autoChooser;

    public AutoSubsystem() {}


    //TODO: TEST THIS
    /**
     * Retrieves the initial pose of an object.
     *
     * @return The initial pose (position and orientation) of the object.
     */
    public Pose2d getCurrentAutoPose() {
        return AutoBuilder.getCurrentPose();
    }
}
