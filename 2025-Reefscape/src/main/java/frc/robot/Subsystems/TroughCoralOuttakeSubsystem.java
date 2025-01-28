package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ThroughCoralOuttakeConstants;

public class TroughCoralOuttakeSubsystem extends SubsystemBase {
    private SparkMax troughMotor = new SparkMax(ThroughCoralOuttakeConstants.kThroughCoralOuttakeMotorPort, MotorType.kBrushless);
    private AbsoluteEncoder troughEncoder;

    public TroughCoralOuttakeSubsystem() {
        troughMotor.configure(
            Configs.TroughCoralOuttakeConfig.troughMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        troughEncoder = troughMotor.getAbsoluteEncoder();
    }

    /**
     * Sets the elevator motors to a specific speed
     * @param speed Speed for the elevator motors
     */
    public void setSpeed(double speed) {
        troughMotor.set(speed);
    }

    /**
     * Gets the value of the elevator encoder
     * @return the Encoder value of the elevator encoder
     */
    public double getEncoder() {
        return troughEncoder.getPosition();
    }
}
