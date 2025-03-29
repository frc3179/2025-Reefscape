/**
 * A command that controls the drive subsystem to move to a specific point using PID controllers.
 *
 * This command calculates the necessary speeds for the drive subsystem to reach the desired positions
 * using PID controllers for each movement direction (drive, rotate, strafe).
 *
 * @param m_DriveSubsystem The drive subsystem to control
 * @param driveGoalPos The goal position for driving
 * @param driveCurrentPos A supplier for the current position for driving
 * @param driveErrOffset The error offset for driving
 * @param rotateGoalPos The goal position for rotation
 * @param rotateCurrentPos A supplier for the current position for rotation
 * @param rotateErrOffset The error offset for rotation
 * @param strafe
 */
package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.DriveSubsystem;

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

    private PIDConstants drivPidConstants;
    private PIDConstants strafePidConstants;
    private PIDConstants rotatePidConstants;

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
            Supplier<Boolean> interupt,
            PIDConstants drivPidConstants,
            PIDConstants strafePidConstants,
            PIDConstants rotatePidConstants
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

        this.drivPidConstants = drivPidConstants;
        this.strafePidConstants = strafePidConstants;
        this.rotatePidConstants = rotatePidConstants;

        drivePidController = new PIDController(this.drivPidConstants.kP, this.drivPidConstants.kI, this.drivPidConstants.kD);
        drivePidController.setSetpoint(this.driveGoalPos);
        drivePidController.setTolerance(this.driveErrOffset);

        rotatePidController = new PIDController(this.rotatePidConstants.kP, this.rotatePidConstants.kI, this.rotatePidConstants.kD);
        rotatePidController.setSetpoint(this.rotateGoalPos);
        rotatePidController.setTolerance(this.rotateErrOffset);

        strafePidController = new PIDController(this.strafePidConstants.kP, this.strafePidConstants.kI, this.strafePidConstants.kD);
        strafePidController.setSetpoint(this.strafeGoalPos);
        strafePidController.setTolerance(this.strafeErrOffset);

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        drivePidController.reset();
        rotatePidController.reset();
        strafePidController.reset();
        SmartDashboard.putBoolean("Full Drive Done", false);
    }

    @Override
    public void execute() {
        double xSpeed = drivePidController.calculate(driveCurrentPos.get());
        double ySpeed = strafePidController.calculate(strafeCurrentPos.get());
        double rotSpeed = rotatePidController.calculate(rotateCurrentPos.get());


        finalXSpeed = MathUtil.clamp(xSpeed, -0.3, 0.3);
        finalYSpeed = MathUtil.clamp(ySpeed, -0.3, 0.3);
        finalRot = MathUtil.clamp(rotSpeed, -0.2, 0.2);

        m_DriveSubsystem.drive(drivePidController.atSetpoint() ? 0.0 : finalXSpeed, strafePidController.atSetpoint() ? 0.0 : -finalYSpeed, rotatePidController.atSetpoint() ? 0.0 : finalRot, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        if (drivePidController.atSetpoint() && strafePidController.atSetpoint() && rotatePidController.atSetpoint()) {
            SmartDashboard.putBoolean("Full Drive Done", true);
            return true;
        }

        return interupt.get();
    }
}
