/**
 * A command that moves the elevator subsystem to a specified position using a PID controller.
 *
 * @param m_elevator The elevator subsystem to control
 * @param goalPos The target position for the elevator
 * @param currentPos A supplier for the current position of the elevator
 * @param errOffset The allowable error offset from the target position
 * @param interupt A supplier for interrupt condition
 * @param P The proportional gain for the PID controller
 * @param I The integral gain for the PID controller
 * @param D The derivative gain for the PID controller
 */
package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;

public class ElevatorMoveToPoint extends Command {
    private final ElevatorSubsystem m_elevator;
    private double goalPos;
    private Supplier<Double> currentPos;
    private double errOffset;
    private Supplier<Boolean> interupt;

    double finalSpeed;

    private PIDController elevatorPidController;
    private double P;
    private double I;
    private double D;
    
    public ElevatorMoveToPoint(
            ElevatorSubsystem m_elevator,
            double goalPos,
            Supplier<Double> currentPos,
            double errOffset,
            Supplier<Boolean> interupt,
            double P,
            double I,
            double D
        ) {

        this.m_elevator = m_elevator;

        this.goalPos = goalPos;
        this.currentPos = currentPos;
        this.errOffset = errOffset;
        this.interupt = interupt;

        this.P = P;
        this.I = I;
        this.D = D;

        elevatorPidController = new PIDController(this.P, this.I, this.D);
        elevatorPidController.setSetpoint(this.goalPos);
        elevatorPidController.setTolerance(this.errOffset);

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        elevatorPidController.reset();
    }

    @Override
    public void execute() {
        finalSpeed = MathUtil.clamp(elevatorPidController.calculate(currentPos.get()), -1, 1);

        m_elevator.setElevatorSpeed(finalSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setElevatorSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        if (elevatorPidController.atSetpoint()) {
            return true;
        }

        return interupt.get();
    }

}
