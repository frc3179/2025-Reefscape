/**
 * The ElevatorSubsystem class controls an elevator system using SparkMax motor controllers and an
 * AbsoluteEncoder for position feedback.
 */
package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor = new SparkMax(ElevatorConstants.kLeftMotorPort, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(ElevatorConstants.kRightMotorPort, MotorType.kBrushless);
    private AbsoluteEncoder elevatorEncoder;
    private double lastEncoderPosition;
    private double rotationCount;

    public ElevatorSubsystem() {
        leftMotor.configure(
            Configs.ElevatorSubsystemConfig.leftElevatorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        
        rightMotor.configure(
            Configs.ElevatorSubsystemConfig.rightElevatorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        elevatorEncoder = rightMotor.getAbsoluteEncoder();

        lastEncoderPosition = 0.0;
        rotationCount = 0.0;

    }

    
    /**
     * Sets the speed of the elevator motor based on the current encoder value.
     *
     * @param speed The speed at which to set the elevator motor
     */
    public void setElevatorSpeed(double speed) {
        if (getEncoder() <= ElevatorConstants.kBottomEncoder) {
           speed = Math.max(speed, 0);
        }

        if (getEncoder() >= ElevatorConstants.kTopEncoder) {
            speed = Math.min(speed, 0);
        }

        leftMotor.set(speed);
    }

    /**
     * Gets the value of the elevator encoder.
     * 
     * @return the Encoder value of the elevator encoder
     */
    public double getEncoder() {
        // if (
        //     elevatorEncoder.getPosition() < lastEncoderPosition &&
        //     (lastEncoderPosition - elevatorEncoder.getPosition()) > ElevatorConstants.kMaxThresholdForResetPercent
        // ) {
        //     rotationCount += 1;
        // } else if (
        //     elevatorEncoder.getPosition() > lastEncoderPosition && 
        //     (elevatorEncoder.getPosition() - lastEncoderPosition) > ElevatorConstants.kMaxThresholdForResetPercent
        // ) {
        //     rotationCount -= 1;
        // }

        // lastEncoderPosition = elevatorEncoder.getPosition();
        // return elevatorEncoder.getPosition() + (rotationCount * ElevatorConstants.kMaxEncoderBeforeReset);
        return elevatorEncoder.getPosition();
    }

    /**
     * Updates the SmartDashboard with the current value of the elevator encoder.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", getEncoder());
    }
}
