package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs.AlgaeSubsystemConfig;
import frc.robot.Constants.AlgaeSubsystemConstants;

public class AlgaeWristSubsystem {
    private SparkMax wristMotor = new SparkMax(AlgaeSubsystemConstants.kWristMotorPort, MotorType.kBrushless);

    public AlgaeWristSubsystem() {
        wristMotor.configure(
            AlgaeSubsystemConfig.wristMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }


    public void setSpeed(double speed) {
        wristMotor.set(speed);
    }
}
