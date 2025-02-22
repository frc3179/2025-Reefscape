package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs.AlgaeSubsystemConfig;
import frc.robot.Constants.AlgaeSubsystemConstants;

public class AlgaeInOutTakeSubsystem {
    private SparkMax inOutTakeMotor = new SparkMax(AlgaeSubsystemConstants.kInOutTakeMotorPort, MotorType.kBrushless);

    public AlgaeInOutTakeSubsystem() {
        inOutTakeMotor.configure(
            AlgaeSubsystemConfig.inOutTakeMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }


    public void setSpeed(double speed) {
        inOutTakeMotor.set(speed);
    }
}
