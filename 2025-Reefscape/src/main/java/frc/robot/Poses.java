package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Poses {
    private static StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
                .getStructTopic("Goal Robot Pose", Pose2d.struct).publish();
        // All Poses are on Blue side
        //TODO
        public static Pose2d topFeedZone = new Pose2d(1.09, 7.07, new Rotation2d(Math.toRadians(-143.73)));
        public static Pose2d bottomFeedZone = new Pose2d(1.09, 1.01, new Rotation2d(Math.toRadians(-35.93)));
        public static Pose2d processor;
        public static Pose2d climbArea;
        public static Pose2d reefA;
        public static Pose2d reefB;
        public static Pose2d reefC;
        public static Pose2d reefD;
        public static Pose2d reefE;
        public static Pose2d reefF;
        public static Pose2d reefG;
        public static Pose2d reefH;
        public static Pose2d reefI;
        public static Pose2d reefJ;
        public static Pose2d reefK;
        public static Pose2d reefL;
    
    
        private static final double xMiddleMeters = 17.54822 / 2; //TODO: TEST
        private static final double yMiddleMeters = 8.05 / 2; //TODO: TEST
    
    
        public static Command fileToCommand(String pathName) {
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
    
        public static Command waypointsToCommand(List<Waypoint> waypoints, GoalEndState endState) {    
            PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
                
            // Create the path using the waypoints created above
            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                    endState // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
    
            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = false;
    
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
        }
    
        public static Command endPoseToCommand(Pose2d endPose, double goalEndVelocity, boolean flip) {
            PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
            FlippingUtil.fieldSizeX = xMiddleMeters * 2;
            FlippingUtil.fieldSizeY = yMiddleMeters * 2;

            if (flip) {
                return AutoBuilder.pathfindToPose(FlippingUtil.flipFieldPose(endPose), constraints, goalEndVelocity);
            } else {
                return AutoBuilder.pathfindToPose(endPose, constraints, goalEndVelocity);
            }
        }
    
    
        // public static Command poseShouldFlipThenToCommand(Pose2d endPose, double goalEndVelocity, boolean shouldFlip) {
        //     if(shouldFlip) {
        //         return endPoseToCommand(flipPoint(endPose), goalEndVelocity);
        //     } else {
        //         return endPoseToCommand(endPose, goalEndVelocity);
        //     }
        // }
}