/**
 * The `AutoSubsystem` class in the `frc.robot.Subsystems` package manages autonomous commands and
 * poses for a robot.
 */
package frc.robot.Subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
