/**
 * The `TroughCoralOuttakeSubsystem` class represents a subsystem in a robot that controls a motor and
 * encoder for a trough coral outtake mechanism.
 */
// package frc.robot.Subsystems;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Configs;
// import frc.robot.Constants.ThroughCoralOuttakeConstants;

// public class TroughCoralOuttakeSubsystem extends SubsystemBase {
//     private SparkMax troughMotor = new SparkMax(ThroughCoralOuttakeConstants.kThroughCoralOuttakeMotorPort, MotorType.kBrushless);
//     private AbsoluteEncoder troughEncoder;

//     public TroughCoralOuttakeSubsystem() {
//         troughMotor.configure(
//             Configs.TroughCoralOuttakeConfig.troughMotorConfig,
//             ResetMode.kResetSafeParameters,
//             PersistMode.kPersistParameters
//         );

//         troughEncoder = troughMotor.getAbsoluteEncoder();
//     }

    
//     /**
//      * Sets the speed of the trough motor.
//      *
//      * @param speed The speed to set for the trough motor
//      */
//     public void setSpeed(double speed) {
//         troughMotor.set(speed);
//     }

    
//     /**
//      * Retrieves the current position of the encoder associated with the trough.
//      *
//      * @return The position of the encoder
//      */
//     public double getEncoder() {
//         return troughEncoder.getPosition();
//     }

//     /**
//      * Updates the SmartDashboard with the current value of the "Through Coral Outtake Encoder" entry.
//      * This method is called periodically to update the SmartDashboard.
//      */
//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Through Coral Outtake Encoder", getEncoder());
//     }
// }
