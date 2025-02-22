package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.BranchCoralOuttakeConfig;
import frc.robot.Constants.BranchCoralOuttakeConstants;
import au.grapplerobotics.LaserCan;

public class BranchCoralOuttakeSubsystem extends SubsystemBase {
    private SparkMax branchMotor = new SparkMax(BranchCoralOuttakeConstants.kBranchCoralOuttakeMotorPort, MotorType.kBrushless);
    private LaserCan frontLaserCan;
    private LaserCan backLaserCan;

    public BranchCoralOuttakeSubsystem() {
        branchMotor.configure(
            BranchCoralOuttakeConfig.branchMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        frontLaserCan = new LaserCan(BranchCoralOuttakeConstants.kFrontLaserCanPort);
        backLaserCan = new LaserCan(BranchCoralOuttakeConstants.kBackLaserCanPort);
    }

    public void setSpeed(double speed) {
        branchMotor.set(speed);
    }

    /**
     * Does not check for unreliable measurements or no measurment. It justs returns the value.
     * @return Measurment in [front, back]
     */
    public LaserCan.Measurement[] getLaserCanValues() {
        LaserCan.Measurement[] finalMeasurement = new LaserCan.Measurement[2];

        finalMeasurement[0] = frontLaserCan.getMeasurement();
        finalMeasurement[1] = backLaserCan.getMeasurement();

        return finalMeasurement;
    }

    @Override
    public void periodic() {
        LaserCan.Measurement[] laserCanValues = getLaserCanValues();


        if (laserCanValues[0] != null && laserCanValues[0].status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SmartDashboard.putNumber("Front Laser Can MM", laserCanValues[0].distance_mm);
        } else {
            SmartDashboard.putNumber("Front Laser Can MM", -999);
            // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
        }


        if (laserCanValues[1] != null && laserCanValues[1].status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SmartDashboard.putNumber("Back Laser Can MM", laserCanValues[1].distance_mm);
        } else {
            SmartDashboard.putNumber("Back Laser Can MM", -999);
            // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
        }
    }
}
