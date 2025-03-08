/**
 * A command that controls the branch intake subsystem.
 * This command sets the speed of the branch intake subsystem and checks if it is finished based on sensor values.
 *
 * @param m_BranchCoralOuttakeSubsystem The branch coral outtake subsystem to control
 * @param interupt A supplier for interruption condition
 */
package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.BranchCoralOuttakeSubsystem;

public class BranchIntake extends Command{
    private BranchCoralOuttakeSubsystem m_BranchCoralOuttakeSubsystem;
    private Supplier<Boolean> interupt;
    private int[] lcValues;

    public BranchIntake(
        BranchCoralOuttakeSubsystem m_BranchCoralOuttakeSubsystem,
        Supplier<Boolean> interupt
    ) {
        this.m_BranchCoralOuttakeSubsystem = m_BranchCoralOuttakeSubsystem;
        this.interupt = interupt;

        lcValues = new int[2];

        addRequirements(m_BranchCoralOuttakeSubsystem);
    }
    

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        lcValues = m_BranchCoralOuttakeSubsystem.getLaserCanMeasurments();

        m_BranchCoralOuttakeSubsystem.setSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (lcValues[0] == 0) {
            return true;
        }

        return interupt.get();
    }
}
