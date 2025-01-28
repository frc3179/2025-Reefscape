package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.TroughCoralOuttakeSubsystem;

public class TeleopTroughCoralOuttake extends Command {

    TroughCoralOuttakeSubsystem m_TroughCoralOuttake;
    Supplier<Double> speed;

    public TeleopTroughCoralOuttake(
        TroughCoralOuttakeSubsystem m_TroughCoralOuttake,
        Supplier<Double> speed
        ) {

        this.m_TroughCoralOuttake = m_TroughCoralOuttake;

        this.speed = speed;

        addRequirements(m_TroughCoralOuttake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_TroughCoralOuttake.setSpeed(speed.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
