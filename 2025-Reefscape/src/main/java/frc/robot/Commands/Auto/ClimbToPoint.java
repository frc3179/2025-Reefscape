/**
 * A command that controls the climbing subsystem to move to a specified position using a PID controller.
 *
 * @param m_climb The climbing subsystem to control
 * @param p The proportional gain for the PID controller
 * @param i The integral gain for the PID controller
 * @param d The derivative gain for the PID controller
 * @param goalPos The target position to reach
 * @param offset The tolerance for considering the goal position reached
 * @param currentPos A supplier for the current position of the climbing subsystem
 * @param interupt A supplier for the interruption condition
 */
package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb.ClimbingSubsystem;

public class ClimbToPoint extends Command{
    private ClimbingSubsystem m_climb;
    private double p;
    private double i;
    private double d;
    private double goalPos;
    private double offset;
    private Supplier<Double> currentPos;
    private Supplier<Boolean> interupt;

    private PIDController climbPID;

    public ClimbToPoint(
        ClimbingSubsystem m_climb,
        double p,
        double i,
        double d,
        double goalPos,
        double offset,
        Supplier<Double> currentPos,
        Supplier<Boolean> interupt
    ) {
        this.m_climb = m_climb;

        this.p = p;
        this.i = i;
        this.d = d;
        this.goalPos = goalPos;
        this.offset = offset;
        this.currentPos = currentPos;
        this.interupt = interupt;

        climbPID = new PIDController(this.p, this.i, this.d);
        climbPID.setSetpoint(this.goalPos);
        climbPID.setTolerance(this.offset);

        addRequirements(m_climb);
    }


    @Override
    public void initialize() {
        climbPID.reset();
    }

    @Override
    public void execute() {
        m_climb.setSpeed(
            MathUtil.clamp(-climbPID.calculate(currentPos.get()), -1, 1)
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return interupt.get() || climbPID.atSetpoint();
    }
}
