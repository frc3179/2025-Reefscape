/**
 * The `AlgaeWristSubsystem` class represents a subsystem for controlling a wrist mechanism using a
 * SparkMax motor controller and an AbsoluteEncoder.
 */
package frc.robot.Subsystems.Algae;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs.AlgaeSubsystemConfig;
import frc.robot.Constants.AlgaeSubsystemConstants;

public class AlgaeWristSubsystem {
    private SparkMax wristMotor = new SparkMax(AlgaeSubsystemConstants.kWristMotorPort, MotorType.kBrushless);
    private AbsoluteEncoder wristEncoder;

    public AlgaeWristSubsystem() {
        wristMotor.configure(
            AlgaeSubsystemConfig.wristMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        wristEncoder = wristMotor.getAbsoluteEncoder();
    }


    /**
     * Sets the speed of the wrist motor.
     *
     * @param speed The speed to set for the wrist motor
     */
    public void setSpeed(double speed) {
        wristMotor.set(speed);
    }

    /**
     * Retrieves the current position of the wrist encoder.
     *
     * @return The position of the wrist encoder
     */
    public double getEncoder() {
        return wristEncoder.getPosition();
    }
}
