/**
 * A command that controls the branch outtake subsystem.
 * This command sets the speed of the branch outtake subsystem to 1.0 and checks if the laser can measurements are zero.
 * If the laser can measurements are zero, it checks for an interrupt signal.
 * 
 * @param m_BranchCoralOuttakeSubsystem The branch coral outtake subsystem to control
 * @param interupt A supplier for the interrupt signal
 */
package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Coral.BranchCoralOuttakeSubsystem;

public class BranchOuttake extends Command{
    private BranchCoralOuttakeSubsystem m_BranchCoralOuttakeSubsystem;
    private Supplier<Boolean> interupt;
    private int[] lcValues;

    public BranchOuttake(
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
        m_BranchCoralOuttakeSubsystem.setSpeed(1.0);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (lcValues[0] == 0) {
            return interupt.get();
        } else {
            return true;
        }

    }
}
