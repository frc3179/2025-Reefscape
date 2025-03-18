/**
 * A command that drives the robot to a specified point using PID control.
 *
 * This command calculates the necessary speeds to drive the robot towards a goal position
 * using a PID controller. It adjusts the robot's movement based on the error between the
 * current position and the goal position.
 *
 * @param m_DriveSubsystem The DriveSubsystem of the robot
 * @param ySpeed A supplier for the forward/backward speed of the robot
 * @param rot A supplier for the rotational speed of the robot
 * @param goalPos The goal position to drive towards
 * @param currentPos A supplier for the current position of the robot
 * @param errOffset The error offset for the PID controller
 * @param interupt
 */
package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrackingConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveDriveToPoint extends Command{
    private DriveSubsystem m_DriveSubsystem;
    private Supplier<Double> ySpeed;
    private Supplier<Double> rot;
    private double goalPos;
    private Supplier<Double> currentPos;
    private double errOffset;
    private Supplier<Boolean> interupt;

    private double finalXSpeed;
    private double finalYSpeed;
    private double finalRot;

    private PIDController drivePidController;

    public DriveDriveToPoint(
            DriveSubsystem m_DriveSubsystem, 
            Supplier<Double> ySpeed,
            Supplier<Double> rot,
            double goalPos,
            Supplier<Double> currentPos,
            double errOffset,
            Supplier<Boolean> interupt
        ){

        this.m_DriveSubsystem = m_DriveSubsystem;

        this.rot = rot;
        this.ySpeed = ySpeed;

        this.currentPos = currentPos;
        this.goalPos = goalPos;
        this.errOffset = errOffset;
        this.interupt = interupt;

        drivePidController = new PIDController(TrackingConstants.kDriveDriveP, TrackingConstants.kDriveDriveI, TrackingConstants.kDriveDriveD);
        drivePidController.setSetpoint(this.goalPos);
        drivePidController.setTolerance(this.errOffset);

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        drivePidController.reset();
    }

    @Override
    public void execute() {
        finalXSpeed = MathUtil.clamp(drivePidController.calculate(currentPos.get()), -0.5, 0.5);
        finalYSpeed = ySpeed.get();
        finalRot = rot.get();

        m_DriveSubsystem.drive(finalXSpeed, finalYSpeed, finalRot, false, false);
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
