/**
 * The `BranchCoralOuttakeSubsystem` class represents a subsystem in a robot that controls a motor and
 * retrieves measurements from laser sensors.
 */
package frc.robot.Subsystems.Coral;

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

    public static final int LC_NULL = -999999999;

    public BranchCoralOuttakeSubsystem() {
        branchMotor.configure(
            BranchCoralOuttakeConfig.branchMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        frontLaserCan = new LaserCan(BranchCoralOuttakeConstants.kFrontLaserCanPort);
        backLaserCan = new LaserCan(BranchCoralOuttakeConstants.kBackLaserCanPort);
    }

    /**
     * Sets the speed of the branch motor.
     *
     * @param speed The speed to set for the branch motor
     */
    public void setSpeed(double speed) {
        branchMotor.set(speed);
    }

    /**
     * Retrieves the measurements from the front and back laser cans.
     *
     * @return [front, back] An array containing the measurements from the front and back laser cans
     */
    public LaserCan.Measurement[] getLaserCanValues() {
        LaserCan.Measurement[] finalMeasurement = new LaserCan.Measurement[2];

        finalMeasurement[0] = frontLaserCan.getMeasurement();
        finalMeasurement[1] = backLaserCan.getMeasurement();

        return finalMeasurement;
    }

    /**
     * Retrieves the laser measurements from the LaserCan device.
     *
     * @return [front, back] An array of two integers representing the distance measurements from two LaserCan devices.
     */
    public int[] getLaserCanMeasurments() {
        int[] res = new int[2];
        LaserCan.Measurement[] lc = getLaserCanValues();
        if (lc[0] == null || lc[0].status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            res[0] = BranchCoralOuttakeSubsystem.LC_NULL;
        } else {
            res[0] = lc[0].distance_mm;
        }


        if (lc[1] == null || lc[1].status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            res[1] = BranchCoralOuttakeSubsystem.LC_NULL;
        } else {
            res[1] = lc[1].distance_mm;
        }


        return res;
    }

    /**
     * Updates the SmartDashboard with the latest measurements from the laser sensors.
     * This method is called periodically to display the front and back laser measurements.
     */
    @Override
    public void periodic() {
        int[] laserCanMeasurments = getLaserCanMeasurments();

        SmartDashboard.putNumber("Front Laser Can MM", laserCanMeasurments[0]);
        SmartDashboard.putNumber("Back Laser Can MM", laserCanMeasurments[1]);
    }
}
