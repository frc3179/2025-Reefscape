/**
 * A command that stops the motor of the Branch Coral Outtake subsystem without interruption.
 * This command sets the speed of the motor to 0.5 during execution and stops it at the end.
 * The command finishes when the first value of the laser measurements array is 0.
 *
 * @param m_BranchCoralOuttakeSubsystem The Branch Coral Outtake subsystem to control
 */
package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.BranchCoralOuttakeSubsystem;

public class BranchIntakeNoInteruptStopMotor extends Command{
    private BranchCoralOuttakeSubsystem m_BranchCoralOuttakeSubsystem;
    private int[] lcValues;

    public BranchIntakeNoInteruptStopMotor(
        BranchCoralOuttakeSubsystem m_BranchCoralOuttakeSubsystem
    ) {
        this.m_BranchCoralOuttakeSubsystem = m_BranchCoralOuttakeSubsystem;

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
    public void end(boolean interrupted) {
        m_BranchCoralOuttakeSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return lcValues[0] == 0;
    }
}
