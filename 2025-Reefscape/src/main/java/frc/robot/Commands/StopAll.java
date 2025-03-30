package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Algae.AlgaeSubsystem;
import frc.robot.Subsystems.Climb.ClimbingSubsystem;
import frc.robot.Subsystems.Coral.BranchCoralOuttakeSubsystem;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;

public class StopAll extends Command {
    private DriveSubsystem m_drive;
    private ElevatorSubsystem m_elevator;
    private AlgaeSubsystem m_AlgaeSubsystem;
    private BranchCoralOuttakeSubsystem m_BranchCoralOuttakeSubsystem;
    private ClimbingSubsystem m_climb;

    public StopAll(
        DriveSubsystem m_drive,
        ElevatorSubsystem m_elevator,
        AlgaeSubsystem m_AlgaeSubsystem,
        BranchCoralOuttakeSubsystem m_BranchCoralOuttakeSubsystem,
        ClimbingSubsystem m_climb
        ) {
            this.m_drive = m_drive;
            this. m_elevator = m_elevator;
            this.m_AlgaeSubsystem = m_AlgaeSubsystem;
            this.m_BranchCoralOuttakeSubsystem = m_BranchCoralOuttakeSubsystem;
            this.m_climb = m_climb;

        addRequirements(m_drive, m_elevator, m_AlgaeSubsystem, m_BranchCoralOuttakeSubsystem, m_climb);
    }

    @Override
    public void initialize() {
        m_drive.drive(0, 0, 0, false, false);
        m_elevator.setElevatorSpeed(0);
        m_AlgaeSubsystem.move(0, 0);
        m_BranchCoralOuttakeSubsystem.setSpeed(0.0);
        m_climb.setSpeed(0);
    }

    @Override
    public void execute() {
        m_drive.drive(0, 0, 0, false, false);
        m_elevator.setElevatorSpeed(0);
        m_AlgaeSubsystem.move(0, 0);
        m_BranchCoralOuttakeSubsystem.setSpeed(0.0);
        m_climb.setSpeed(0);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, false, false);
        m_elevator.setElevatorSpeed(0);
        m_AlgaeSubsystem.move(0, 0);
        m_BranchCoralOuttakeSubsystem.setSpeed(0.0);
        m_climb.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
