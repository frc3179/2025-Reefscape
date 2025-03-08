/**
 * A command that moves the Trough Coral Outtake subsystem to a specified position using a PID controller.
 *
 * This command calculates the necessary speed to reach the goal position based on the current position
 * and updates the Trough Coral Outtake subsystem accordingly.
 *
 * @param m_troughCoralOuttake The Trough Coral Outtake subsystem to control
 * @param goalPos The target position to reach
 * @param currentPos A supplier for the current position of the subsystem
 * @param errOffset The allowable error offset from the target position
 * @param interupt A supplier for the interruption condition
 */
package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrackingConstants;
import frc.robot.Subsystems.TroughCoralOuttakeSubsystem;

public class ThroughCoralOuttakeToPoint extends Command{
    private TroughCoralOuttakeSubsystem m_troughCoralOuttake;
    private double goalPos;
    private Supplier<Double> currentPos;
    private double errOffset;
    private Supplier<Boolean> interupt;

    private double finalSpeed;

    private PIDController throughCoralOuttakePID;

    public ThroughCoralOuttakeToPoint(
            TroughCoralOuttakeSubsystem m_troughCoralOuttake,
            double goalPos,
            Supplier<Double> currentPos,
            double errOffset,
            Supplier<Boolean> interupt
        ){

        this.m_troughCoralOuttake = m_troughCoralOuttake;

        this.currentPos = currentPos;
        this.goalPos = goalPos;
        this.errOffset = errOffset;
        this.interupt = interupt;

        throughCoralOuttakePID = new PIDController(TrackingConstants.kThroughCoralOuttakeP, TrackingConstants.kThroughCoralOuttakeI, TrackingConstants.kThroughCoralOuttakeD);
        throughCoralOuttakePID.setSetpoint(this.goalPos);
        throughCoralOuttakePID.setTolerance(this.errOffset);

        addRequirements(m_troughCoralOuttake);
    }

    @Override
    public void initialize() {
        throughCoralOuttakePID.reset();
    }

    @Override
    public void execute() {
        finalSpeed = MathUtil.clamp(throughCoralOuttakePID.calculate(currentPos.get()), -0.5, 0.5);

        m_troughCoralOuttake.setSpeed(finalSpeed);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (throughCoralOuttakePID.atSetpoint()) {
            return true;
        }

        return interupt.get();
    }
}
