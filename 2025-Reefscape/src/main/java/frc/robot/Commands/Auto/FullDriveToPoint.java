package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrackingConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class FullDriveToPoint extends Command{
    private DriveSubsystem m_DriveSubsystem;

    private double driveGoalPos;
    private Supplier<Double> driveCurrentPos;
    private double driveErrOffset;
    private double rotateGoalPos;
    private Supplier<Double> rotateCurrentPos;
    private double rotateErrOffset;
    private double strafeGoalPos;
    private Supplier<Double> strafeCurrentPos;
    private double strafeErrOffset;

    private Supplier<Boolean> interupt;

    private double finalXSpeed;
    private double finalYSpeed;
    private double finalRot;

    private PIDController drivePidController;
    private PIDController rotatePidController;
    private PIDController strafePidController;

    public FullDriveToPoint(
            DriveSubsystem m_DriveSubsystem,
            double driveGoalPos,
            Supplier<Double> driveCurrentPos,
            double driveErrOffset,
            double rotateGoalPos,
            Supplier<Double> rotateCurrentPos,
            double rotateErrOffset,
            double strafeGoalPos,
            Supplier<Double> strafeCurrentPos,
            double strafeErrOffset,
            Supplier<Boolean> interupt
        ){

        this.m_DriveSubsystem = m_DriveSubsystem;

        this.driveGoalPos = driveGoalPos;
        this.driveCurrentPos = driveCurrentPos;
        this.driveErrOffset = driveErrOffset;

        this.rotateGoalPos = rotateGoalPos;
        this.rotateCurrentPos = rotateCurrentPos;
        this.rotateErrOffset = rotateErrOffset;

        this.strafeGoalPos = strafeGoalPos;
        this.strafeCurrentPos = strafeCurrentPos;
        this.strafeErrOffset = strafeErrOffset;

        this.interupt = interupt;

        drivePidController = new PIDController(TrackingConstants.kDriveDriveP, TrackingConstants.kDriveDriveI, TrackingConstants.kDriveDriveD);
        drivePidController.setSetpoint(this.driveGoalPos);
        drivePidController.setTolerance(this.driveErrOffset);

        rotatePidController = new PIDController(TrackingConstants.kRotateDriveP, TrackingConstants.kRotateDriveI, TrackingConstants.kRotateDriveD);
        rotatePidController.setSetpoint(this.rotateGoalPos);
        rotatePidController.setTolerance(this.rotateErrOffset);

        strafePidController = new PIDController(TrackingConstants.kStrafeDriveP, TrackingConstants.kStrafeDriveI, TrackingConstants.kStrafeDriveD);
        strafePidController.setSetpoint(this.strafeGoalPos);
        strafePidController.setTolerance(this.strafeErrOffset);

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        drivePidController.reset();
        rotatePidController.reset();
        strafePidController.reset();
    }

    @Override
    public void execute() {
        finalXSpeed = MathUtil.clamp(drivePidController.calculate(driveCurrentPos.get()), -0.5, 0.5);
        finalYSpeed = MathUtil.clamp(strafePidController.calculate(strafeCurrentPos.get()), -0.5, 0.5);
        finalRot = MathUtil.clamp(rotatePidController.calculate(rotateCurrentPos.get()), -0.5, 0.5);

        m_DriveSubsystem.drive(finalXSpeed, finalYSpeed, finalRot, false);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (drivePidController.atSetpoint()) {
            return true;
        }

        return interupt.get();
    }
}
