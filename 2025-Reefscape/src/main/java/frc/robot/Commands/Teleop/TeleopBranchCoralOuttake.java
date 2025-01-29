package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.BranchCoralOuttakeSubsystem;

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
