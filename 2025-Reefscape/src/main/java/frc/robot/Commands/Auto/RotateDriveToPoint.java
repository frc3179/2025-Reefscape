/**
 * A command that rotates the drive subsystem to a specified point using PID control.
 *
 * This command calculates the necessary rotation to reach a goal position using a PID controller.
 * The command ends when the rotation reaches the goal position or when interrupted.
 *
 * @param m_DriveSubsystem The DriveSubsystem to control
 * @param xSpeed A supplier for the x-axis speed
 * @param ySpeed A supplier for the y-axis speed
 * @param goalPos The goal position to rotate towards
 * @param currentPos A supplier for the current position
 * @param errOffset The error offset tolerance for the PID controller
 * @param interupt A supplier for the interruption condition
 */
package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrackingConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class RotateDriveToPoint extends Command{
    private DriveSubsystem m_DriveSubsystem;
    private Supplier<Double> xSpeed;
    private Supplier<Double> ySpeed;
    private double goalPos;
    private Supplier<Double> currentPos;
    private double errOffset;
    private Supplier<Boolean> interupt;

    private double finalXSpeed;
    private double finalYSpeed;
    private double finalRot;

    private PIDController rotatePidController;

    public RotateDriveToPoint(
            DriveSubsystem m_DriveSubsystem,
            Supplier<Double> xSpeed, 
            Supplier<Double> ySpeed,
            double goalPos,
            Supplier<Double> currentPos,
            double errOffset,
            Supplier<Boolean> interupt
        ){

        this.m_DriveSubsystem = m_DriveSubsystem;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;

        this.currentPos = currentPos;
        this.goalPos = goalPos;
        this.errOffset = errOffset;
        this.interupt = interupt;

        rotatePidController = new PIDController(TrackingConstants.kRotateDriveP, TrackingConstants.kRotateDriveI, TrackingConstants.kRotateDriveD);
        rotatePidController.setSetpoint(this.goalPos);
        rotatePidController.setTolerance(this.errOffset);

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        rotatePidController.reset();
    }

    @Override
    public void execute() {
        finalXSpeed = xSpeed.get();
        finalYSpeed = ySpeed.get();
        //finalRot = MathUtil.clamp(rotatePidController.calculate(currentPos.get()), -1, 1);
        finalRot = rotatePidController.calculate(currentPos.get());

        m_DriveSubsystem.drive(finalXSpeed, finalYSpeed, finalRot, false, false);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (rotatePidController.atSetpoint()) {
            return true;
        }

        return interupt.get();
    }
}
