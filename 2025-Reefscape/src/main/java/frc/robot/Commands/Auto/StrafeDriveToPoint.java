package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrackingConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class StrafeDriveToPoint extends Command{
    private DriveSubsystem m_DriveSubsystem;
    private Supplier<Double> xSpeed;
    private Supplier<Double> rot;

    private double goalPos;
    private Supplier<Double> currentPos;
    private double errOffset;
    private Supplier<Boolean> interupt;

    private double finalXSpeed;
    private double finalYSpeed;
    private double finalRot;

    private PIDController strafePidController;

    public StrafeDriveToPoint(
            DriveSubsystem m_DriveSubsystem,
            Supplier<Double> xSpeed, 
            Supplier<Double> rot,
            double goalPos,
            Supplier<Double> currentPos,
            double errOffset,
            Supplier<Boolean> interupt
        ){

        this.m_DriveSubsystem = m_DriveSubsystem;

        this.xSpeed = xSpeed;
        this.rot = rot;

        this.currentPos = currentPos;
        this.goalPos = goalPos;
        this.errOffset = errOffset;
        this.interupt = interupt;

        strafePidController = new PIDController(TrackingConstants.kStrafeDriveP, TrackingConstants.kStrafeDriveI, TrackingConstants.kStrafeDriveD);
        strafePidController.setSetpoint(this.goalPos);
        strafePidController.setTolerance(this.errOffset);

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

        m_DriveSubsystem.drive(finalXSpeed, finalYSpeed, finalRot, false, false);
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
