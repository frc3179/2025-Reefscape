package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {
    private Spark blinkin;
    private boolean isBlueAlliance;

    public static final double RED = 0.0; //color values for the blikin in the set method
    public static final double BLUE = 0.0;
    public static final double GREEN = 0.0;
    public static final double YELLOW = 0.0;
    
    public LightSubsystem(int blinkinPort) {
        blinkin = new Spark(blinkinPort);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isBlueAlliance = (alliance.get() == DriverStation.Alliance.Blue);
        } else {
            isBlueAlliance = true;
        }
    }

    public void set(double value) {
        blinkin.set(value);
    }

    public void setToAllianceColor() {
        if (isBlueAlliance) {
            set(BLUE);
        } else {
            set(RED);
        }
    }

    /**
     * 
     * @param frontLcValue
     * @param backLcValue
     * @return the color for the robot. GREEN = ALL can see; YELLOW = ONE can see; BLUE/RED = NONE can see
     */
    public double laserCanToColor(int frontLcValue, int backLcValue) {
        boolean canFrontSee = frontLcValue != BranchCoralOuttakeSubsystem.LC_NULL;
        boolean canBackSee = backLcValue != BranchCoralOuttakeSubsystem.LC_NULL;

        if (canFrontSee && canBackSee) {
            return LightSubsystem.GREEN;
        } else if (canFrontSee) {
            return LightSubsystem.YELLOW;
        } else if (canBackSee) {
            return LightSubsystem.YELLOW;
        } else {
            return isBlueAlliance ? LightSubsystem.BLUE : LightSubsystem.RED;
        }
    }
}