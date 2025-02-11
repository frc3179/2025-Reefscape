package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.BranchCoralOuttakeConfig;
import frc.robot.Constants.BranchCoralOuttakeConstants;

public class BranchCoralOuttakeSubsystem extends SubsystemBase {
    private SparkMax branchMotor = new SparkMax(BranchCoralOuttakeConstants.kBranchCoralOuttakeMotorPort, MotorType.kBrushless);

    public BranchCoralOuttakeSubsystem() {
        branchMotor.configure(
            BranchCoralOuttakeConfig.branchMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void setSpeed(double speed) {
        branchMotor.set(speed);
    }
}
