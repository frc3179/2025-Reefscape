/**
 * The `AlgaeInOutTakeSubsystem` class represents a subsystem for controlling an intake/outtake
 * mechanism using a SparkMax motor controller.
 */
package frc.robot.Subsystems.Algae;

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


    /**
     * Sets the speed of the motor controlling the intake/outtake mechanism.
     *
     * @param speed The speed at which to set the motor
     */
    public void setSpeed(double speed) {
        inOutTakeMotor.set(speed);
    }
}
