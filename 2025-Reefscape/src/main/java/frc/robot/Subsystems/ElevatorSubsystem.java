package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor = new SparkMax(ElevatorConstants.kLeftMotorPort, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(ElevatorConstants.kRightMotorPort, MotorType.kBrushless);
    private DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(ElevatorConstants.kEncoderPort);

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
    }

    /**
     * Sets the elevator motors to a specific speed
     * @param speed Speed for the elevator motors
     */
    public void setElevatorSpeed(double speed) {
        rightMotor.set(speed);
    }

    /**
     * Gets the value of the elevator encoder
     * @return the Encoder value of the elevator encoder
     */
    public double getEncoder() {
        return elevatorEncoder.get();
    }
}
