package frc.robot.Subsystems;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {
    SendableChooser<Command> autoChooser;

    public AutoSubsystem(SendableChooser<Command> autoChooser) {
        this.autoChooser = autoChooser;
    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }

    //TODO: CHECK THIS
    public Pose2d getInitPose(Path JSONpath) {
        Trajectory trajectory = new Trajectory();
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(JSONpath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to get Init Pose of Robot at: " + JSONpath, e.getStackTrace());
        }

        return trajectory.getInitialPose();
    }

}
