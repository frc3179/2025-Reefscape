/**
 * A command to control the lights on the robot during teleoperation.
 * This command sets the color of the lights based on the provided color supplier.
 *
 * @param m_LightSubsystem The subsystem responsible for controlling the lights.
 * @param color A supplier that provides the color value for the lights.
 */
package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Lights.LightSubsystem;

public class TeleopLights extends Command {
    private LightSubsystem m_LightSubsystem;
    private Supplier<Double> color;

    public TeleopLights(
        LightSubsystem m_LightSubsystem,
        Supplier<Double> color
    ) {
        this.m_LightSubsystem = m_LightSubsystem;

        this.color = color;

        addRequirements(m_LightSubsystem);
    }


    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_LightSubsystem.set(color.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
