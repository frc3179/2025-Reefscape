package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BranchCoralOuttakeConstants;

public class BranchCoralOuttakeSubsystem extends SubsystemBase {
    private Spark branchMotor = new Spark(BranchCoralOuttakeConstants.kBranchCoralOuttakeMotorPort);

    public BranchCoralOuttakeSubsystem() {
        branchMotor.setInverted(false);
    }

    public void setSpeed(double speed) {
        branchMotor.set(speed);
    }
}
