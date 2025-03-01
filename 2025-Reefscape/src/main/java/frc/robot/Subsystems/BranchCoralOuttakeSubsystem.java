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

    /**
     * 
     * @return [front, back]
     */
    public int[] getLaserCanMeasurments() {
        int[] res = new int[2];
        LaserCan.Measurement[] lc = getLaserCanValues();
        if (lc[0].status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            res[0] = lc[0].distance_mm;
        } else {
            res[0] = BranchCoralOuttakeSubsystem.LC_NULL;
        }


        if (lc[1].status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            res[1] = lc[1].distance_mm;
        } else {
            res[1] = BranchCoralOuttakeSubsystem.LC_NULL;
        }


        return res;
    }

    @Override
    public void periodic() {
        int[] laserCanMeasurments = getLaserCanMeasurments();

        SmartDashboard.putNumber("Front Laser Can MM", laserCanMeasurments[0]);
        SmartDashboard.putNumber("Back Laser Can MM", laserCanMeasurments[1]);
    }
}
