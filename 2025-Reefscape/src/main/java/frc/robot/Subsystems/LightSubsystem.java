/**
 * The LightSubsystem class in Java controls a Blinkin LED driver and determines the color of a laser
 * based on sensor values and alliance color.
 */
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {
    private Spark blinkin;
    private boolean isBlueAlliance;

    public static final double RED = 0.61; //color values for the blikin in the set method
    public static final double BLUE = 0.87;
    public static final double GREEN = 0.77;
    public static final double YELLOW = 0.67;
    
    public LightSubsystem(int blinkinPort) {
        blinkin = new Spark(blinkinPort);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isBlueAlliance = (alliance.get() == DriverStation.Alliance.Blue);
        } else {
            isBlueAlliance = false;
        }
    }

    /**
     * Sets the value of the Blinkin LED driver.
     *
     * @param value The value to set for the Blinkin LED driver.
     */
    public void set(double value) {
        blinkin.set(value);
    }

    /**
     * Sets the color to the alliance color, either blue or red.
     * If the alliance is blue, the color is set to blue; otherwise, it is set to red.
     */
    public void setToAllianceColor() {
        if (isBlueAlliance) {
            set(BLUE);
        } else {
            set(RED);
        }
    }

    
    /**
     * Determines the color of the laser based on the front and back sensor values.
     *
     * @param frontLcValue The value of the front sensor
     * @param backLcValue The value of the back sensor
     * @return The color of the laser: GREEN if both sensors see, YELLOW if only one sensor sees, BLUE if on the blue alliance, otherwise RED
     */
    public double laserCanToColor(int frontLcValue, int backLcValue) {
        boolean canFrontSee = frontLcValue == 0;
        boolean canBackSee = backLcValue == 0;

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