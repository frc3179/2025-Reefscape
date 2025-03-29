/**
 * A command that allows teleoperation of the Algae subsystem.
 * This command controls the movement of the Algae subsystem based on input speeds for intake/outtake and wrist.
 *
 * @param m_AlgaeSubsystem The AlgaeSubsystem to control
 * @param inOutTakeSpeed A supplier for the speed of the intake/outtake mechanism
 * @param wristSpeed A supplier for the speed of the wrist mechanism
 */
package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Algae.AlgaeSubsystem;

public class TeleopAlgae extends Command {
    private AlgaeSubsystem m_AlgaeSubsystem;
    private Supplier<Double> inOutTakeSpeed;
    private Supplier<Double> wristSpeed;

    public TeleopAlgae(
        AlgaeSubsystem m_AlgaeSubsystem,
        Supplier<Double> inOutTakeSpeed,
        Supplier<Double> wristSpeed
        ) {

        this.m_AlgaeSubsystem = m_AlgaeSubsystem;

        this.inOutTakeSpeed = inOutTakeSpeed;
        this.wristSpeed = wristSpeed;

        addRequirements(m_AlgaeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_AlgaeSubsystem.move(inOutTakeSpeed.get(), wristSpeed.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
