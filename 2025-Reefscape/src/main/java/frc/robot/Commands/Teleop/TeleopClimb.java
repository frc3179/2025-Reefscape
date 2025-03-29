/**
 * A command that allows teleoperated control of the climbing subsystem.
 * This command sets the speed of the climbing mechanism based on the provided supplier.
 *
 * @param m_climb The ClimbingSubsystem to control
 * @param speed A supplier providing the speed value for the climbing mechanism
 */
package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb.ClimbingSubsystem;

public class TeleopClimb extends Command {
    private ClimbingSubsystem m_climb;
    private Supplier<Double> speed;
    
    public TeleopClimb(
        ClimbingSubsystem m_climb,
        Supplier<Double> speed
    ){
        this.m_climb = m_climb;
        
        this.speed = speed;

        addRequirements(m_climb);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_climb.setSpeed(speed.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
