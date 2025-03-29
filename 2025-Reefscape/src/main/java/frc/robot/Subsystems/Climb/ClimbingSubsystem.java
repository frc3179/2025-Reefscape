/**
 * The `ClimbingSubsystem` class represents a subsystem for controlling a climbing mechanism on a
 * robot, including a motor and an encoder.
 */
package frc.robot.Subsystems.Climb;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ClimbingSubsystemConfig;
import frc.robot.Constants.ClimbSubsystemConstants;

public class ClimbingSubsystem extends SubsystemBase {
    private SparkMax climbMotor = new SparkMax(ClimbSubsystemConstants.kClimbMotorPort, MotorType.kBrushless);
    private AbsoluteEncoder climbEncoder;

    public ClimbingSubsystem() {
        climbMotor.configure(
            ClimbingSubsystemConfig.climbMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        climbEncoder = climbMotor.getAbsoluteEncoder();
    }

    
    /**
     * Sets the speed of the climb motor.
     *
     * @param speed The speed to set for the climb motor
     */
    public void setSpeed(double speed) {
        climbMotor.set(speed);
    }

    
    /**
     * Retrieves the current position of the encoder used for climbing.
     *
     * @return The position of the climbing encoder
     */
    public double getEncoder() {
        return climbEncoder.getPosition();
    }


    /**
     * The `periodic` function in Java periodically updates the SmartDashboard with the current value
     * of the climb encoder.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Encoder", getEncoder());
    }
}
