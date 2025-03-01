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

    public void setValues(SendableChooser<Command> autoChooser) {
        this.autoChooser = autoChooser;
    }

    public Command getAuto() {
        startPose = AutoBuilder.getCurrentPose();
        return autoChooser.getSelected();
    }

    //TODO: TEST THIS
    public Pose2d getInitPose() {
        return startPose;
    }

}
