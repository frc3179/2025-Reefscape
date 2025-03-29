/**
 * The `AlgaeSubsystem` class in Java controls the intake/outtake and wrist mechanisms of a robotic
 * system and updates the SmartDashboard with the encoder value of the wrist mechanism.
 */
package frc.robot.Subsystems.Algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    /**
     * The `move` function sets the speed of the intake/outtake mechanism and the wrist mechanism in a
     * Java program.
     * 
     * @param inOutTakeSpeed The `inOutTakeSpeed` parameter likely represents the speed at which a
     * mechanism for intake and outtake is moving. This could be used to control the speed at which an
     * object is being taken in or released by the mechanism.
     * 
     * @param wristSpeed The `wristSpeed` parameter in the `move` method likely represents the speed at
     * which the wrist component of a robotic system should move. This parameter is used to control the
     * speed of the wrist movement when the `move` method is called.
     */
    public void move(double inOutTakeSpeed, double wristSpeed) {
        m_InOutTake.setSpeed(inOutTakeSpeed);
        m_Wrist.setSpeed(wristSpeed);
    }

    /**
     * Updates the SmartDashboard with the current encoder value of the wrist mechanism.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Encoder value", m_Wrist.getEncoder());
    }
}
