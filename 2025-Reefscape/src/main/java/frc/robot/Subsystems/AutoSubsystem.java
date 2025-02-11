package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {
    SendableChooser<Command> autoChooser;
    private Pose2d startPose;

    public AutoSubsystem() {}

    public void setValues(SendableChooser<Command> autoChooser) {
        this.autoChooser = autoChooser;
    }

    //TODO: FIND HOW THE NAME CORRELATES TO THE JSON PATH COULD ONLY BE BLUE AND NOT UPDATE FOR RED
    public Command getAuto() {
        // DriverStation.reportError("NOT AN ERROR; AUTO NAME = " + autoChooser.getSelected().toString(), null);
        // startPose = AutoBuilder.getCurrentPose();
        return autoChooser.getSelected();
    }

    //TODO: TEST THIS
    public Pose2d getInitPose() {
        return startPose;
        //return AutoBuilder.getCurrentPose();
        //return getInitPoseFromJSON(null);
    }

    //TODO: CHECK THIS
    // private Pose2d getInitPoseFromJSON(Path JSONpath) {
    //     Trajectory trajectory = new Trajectory();
    //     try {
    //         trajectory = TrajectoryUtil.fromPathweaverJson(JSONpath);
    //     } catch (IOException e) {
    //         DriverStation.reportError("Unable to get Init Pose of Robot at: " + JSONpath, e.getStackTrace());
    //     }

    //     return trajectory.getInitialPose();
    // }

}
