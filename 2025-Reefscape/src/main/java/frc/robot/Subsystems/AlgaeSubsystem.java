package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    private AlgaeInOutTakeSubsystem m_InOutTake;
    private AlgaeWristSubsystem m_Wrist;

    public AlgaeSubsystem(
        AlgaeInOutTakeSubsystem m_AlgaeInOutTakeSubsystem,
        AlgaeWristSubsystem m_AlgaeWristSubsystem
    ) {
        this.m_InOutTake = m_AlgaeInOutTakeSubsystem;
        this.m_Wrist = m_AlgaeWristSubsystem;
    }

    public void move(double inOutTakeSpeed, double wristSpeed) {
        m_InOutTake.setSpeed(inOutTakeSpeed);
        m_Wrist.setSpeed(wristSpeed);
    }
}
