package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrackingConstants;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorMoveToPoint extends Command {
    private final ElevatorSubsystem m_elevator;
    private double goalPos;
    private Supplier<Double> currentPos;
    private double errOffset;
    private Supplier<Boolean> interupt;

    double finalSpeed;

    private PIDController elevatorPidController;
    
    public ElevatorMoveToPoint(
            ElevatorSubsystem m_elevator,
            double goalPos,
            Supplier<Double> currentPos,
            double errOffset,
            Supplier<Boolean> interupt
        ) {

        this.m_elevator = m_elevator;

        this.goalPos = goalPos;
        this.currentPos = currentPos;
        this.errOffset = errOffset;
        this.interupt = interupt;

        elevatorPidController = new PIDController(TrackingConstants.kElevatorP, TrackingConstants.kElevatorI, TrackingConstants.kElevatorD);
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
        finalSpeed = MathUtil.clamp(elevatorPidController.calculate(currentPos.get()), -0.5, 0.5); //TODO: change the speed

        m_elevator.setElevatorSpeed(finalSpeed);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (elevatorPidController.atSetpoint()) {
            return true;
        }

        return interupt.get();
    }

}
