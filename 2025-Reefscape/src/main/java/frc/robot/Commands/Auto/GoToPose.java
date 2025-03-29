package frc.robot.Commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.DriveSubsystem;

public class GoToPose extends Command {
    private DriveSubsystem drive;
    private Pose2d pose;

    private Command go;

    public GoToPose(DriveSubsystem drive, Pose2d pose) {
        this.drive = drive;
        this.pose = pose;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        go = drive.followPathToPose(pose, drive.shouldFlipPath());
        go.schedule();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0,0,0,false,false);
    }
}