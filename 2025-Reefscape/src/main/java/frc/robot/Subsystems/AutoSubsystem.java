package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
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

    //TODO: DO THIS
    // public Pose2d getInitPose() {

    // }

}
