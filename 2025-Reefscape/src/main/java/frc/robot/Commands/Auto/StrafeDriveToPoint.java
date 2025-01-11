package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrackingConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class StrafeDriveToPoint extends Command{
    DriveSubsystem m_DriveSubsystem;
    Supplier<Double> xSpeed;
    Supplier<Double> ySpeed;
    Supplier<Double> rot;
    Supplier<Boolean> fieldRelative;

    double goalPos;
    Supplier<Double> currentPos;
    double errOffset;
    Supplier<Boolean> interupt;

    double finalXSpeed;
    double finalYSpeed;
    double finalRot;

    PIDController strafePidController;

    public StrafeDriveToPoint(
            DriveSubsystem m_DriveSubsystem,
            Supplier<Double> xSpeed, 
            Supplier<Double> ySpeed, 
            Supplier<Double> rot, 
            Supplier<Boolean> fieldRelative,
            double goalPos,
            Supplier<Double> currentPos,
            double errOffset,
            Supplier<Boolean> interupt
        ){

        this.m_DriveSubsystem = m_DriveSubsystem;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.fieldRelative = fieldRelative;

        this.currentPos = currentPos;
        this.goalPos = goalPos;
        this.errOffset = errOffset;
        this.interupt = interupt;

        strafePidController = new PIDController(TrackingConstants.kStrafeDriveP, TrackingConstants.kStrafeDriveI, TrackingConstants.kStrafeDriveD);
        strafePidController.setSetpoint(goalPos);
        strafePidController.setTolerance(errOffset);

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        strafePidController.reset();
    }

    @Override
    public void execute() {
        finalXSpeed = xSpeed.get();
        finalYSpeed = MathUtil.clamp(strafePidController.calculate(currentPos.get()), -0.5, 0.5);
        finalRot = rot.get();

        m_DriveSubsystem.drive(finalXSpeed, finalYSpeed, finalRot, fieldRelative.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (strafePidController.atSetpoint()) {
            return true;
        }

        return interupt.get();
    }
}
