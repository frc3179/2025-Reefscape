/**
 * A command to control the Branch Coral Outtake subsystem during teleop.
 * This command sets the speed of the outtake based on a supplier function.
 *
 * @param m_BranchCoralOuttake The BranchCoralOuttakeSubsystem to control
 * @param speed A supplier function that provides the speed value
 */
package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Coral.BranchCoralOuttakeSubsystem;

public class TeleopBranchCoralOuttake extends Command {
    private BranchCoralOuttakeSubsystem m_BranchCoralOuttake;
    private Supplier<Double> speed;

    public TeleopBranchCoralOuttake(
        BranchCoralOuttakeSubsystem m_BranchCoralOuttake,
        Supplier<Double> speed
        ) {

        this.m_BranchCoralOuttake = m_BranchCoralOuttake;

        this.speed = speed;

        addRequirements(m_BranchCoralOuttake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_BranchCoralOuttake.setSpeed(speed.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
