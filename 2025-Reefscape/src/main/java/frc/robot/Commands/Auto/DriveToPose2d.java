package frc.robot.Commands.Auto;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.TrackingSubsystem;

public class DriveToPose2d extends Command{
    TrackingSubsystem m_TrackingSubsystem;
    DriveSubsystem m_DriveSubsystem;

    Pose2d targetPose;
    Supplier<Boolean> interupt;

    List<Waypoint> waypoints;
    PathConstraints constraints;
    PathPlannerPath goalPath;

    public DriveToPose2d(
            TrackingSubsystem m_TrackingSubsystem,
            DriveSubsystem m_DriveSubsystem,
            Pose2d targetPose,
            Supplier<Boolean> interupt
        ){

        this.m_TrackingSubsystem = m_TrackingSubsystem;
        this.m_DriveSubsystem = m_DriveSubsystem;

        this.targetPose = targetPose;
        this.interupt = interupt;

        addRequirements(m_TrackingSubsystem, m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        //The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        waypoints = PathPlannerPath.waypointsFromPoses(
            m_TrackingSubsystem.poseEstimator.getEstimatedPosition()
        );

        constraints = PathConstraints.unlimitedConstraints(12.0);

        //TODO: TEST NULL FOR WAYPOINTS
        goalPath = new PathPlannerPath(
            null,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths
            new GoalEndState(0.0, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation
        );


    }

    @Override
    public void execute() {
        //TODO: TEST
        new SequentialCommandGroup(
            AutoBuilder.followPath(goalPath)
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (m_TrackingSubsystem.poseEstimator.getEstimatedPosition().equals(targetPose)) {
            return true;
        }
        return interupt.get();
    }
}
