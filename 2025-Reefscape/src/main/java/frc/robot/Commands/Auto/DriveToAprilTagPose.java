package frc.robot.Commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.Poses;

public class DriveToAprilTagPose extends Command{
    private DriveSubsystem drive;
    private int aprilTagId;
    // -1 = left; 1 = right (default so bassically right != -1);
    private int side;

    // [left, right]
    private Pose2d[] possiblePoses;

    private Command go;

    public DriveToAprilTagPose(
        DriveSubsystem drive,
        int aprilTagId,
        int side
    ) {
        this.drive = drive;
        this. aprilTagId = aprilTagId;
        this.side = side;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        aprilTagId = Poses.redSideAprilTagIdToBlueSideId(aprilTagId);
        possiblePoses = Poses.aprilTagIdToPoseOptions(aprilTagId);

        if (side == -1) {
            go = drive.followPathToPose(possiblePoses[0], drive.shouldFlipPath());
            go.schedule();
        } else {
            go = drive.followPathToPose(possiblePoses[1], drive.shouldFlipPath());
            go.schedule();
        }
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
