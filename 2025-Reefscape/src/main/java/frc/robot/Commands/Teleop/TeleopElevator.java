/**
 * A command to control the elevator subsystem during teleoperation.
 * This command sets the speed of the elevator based on the provided supplier.
 *
 * @param m_elevator The elevator subsystem to control
 * @param speed A supplier providing the speed value for the elevator
 */
package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class TeleopElevator extends Command {
    private final ElevatorSubsystem m_elevator;
    
    private final Supplier<Double> speed;

    public TeleopElevator(
        ElevatorSubsystem m_elevator,
        Supplier<Double> speed
        ) {

        this.m_elevator = m_elevator;

        this.speed = speed;

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_elevator.setElevatorSpeed(speed.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
    
