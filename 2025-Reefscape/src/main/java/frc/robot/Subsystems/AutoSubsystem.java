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

    
    public Command fileToCommand(String pathName) {
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            // Create a path following command using AutoBuilder. This will also trigger event markers.
            
            PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
        } catch (Exception e) {
            DriverStation.reportError("Error converting file to command: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command waypointsToCommand(List<Waypoint> waypoints, GoalEndState endState) {    
        PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
            
        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                endState // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }

    public Command endPoseToCommand(Pose2d endPose, double goalEndVelocity) {
        PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
        
        return AutoBuilder.pathfindToPose(endPose, constraints, goalEndVelocity);
    }

}
