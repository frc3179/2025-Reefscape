/**
 * The `DriveSpeedSettings` class in the `SpeedSettings` package contains methods for calculating final
 * speed based on mode settings and converting Point of View (POV) angles to speed components.
 */
package frc.robot.SpeedSettings;

public class DriveSpeedSettings {
    private double slowSpeedPCT;
    private double defaultSpeedPCT;
    private double fastSpeedPCT;

    public DriveSpeedSettings(double slowSpeedPCT, double defaultSpeedPCT, double fastSpeedPCT) {
        this.slowSpeedPCT = slowSpeedPCT;
        this.defaultSpeedPCT = defaultSpeedPCT;
        this.fastSpeedPCT = fastSpeedPCT;
    }

    /**
     * Calculates the final speed based on the base speed and mode settings.
     *
     * @param baseSpeed The base speed value
     * @param isFastMode A boolean indicating if fast mode is enabled
     * @param isSlowMode A boolean indicating if slow mode is enabled
     * @return The final speed value after applying the appropriate adjustments
     */
    public double getFinalSpeed(double baseSpeed, boolean isFastMode, boolean isSlowMode) {
        if (isSlowMode) {
            return slowSpeedPCT * baseSpeed;
        } else if (isFastMode) {
            return fastSpeedPCT * baseSpeed;
        }
        return defaultSpeedPCT * baseSpeed;
    }

    
    /**
     * Converts Point of View (POV) angle to speed components in x and y directions.
     *
     * @param POV The angle of the point of view in degrees
     * @return An array containing the speed components [x, y] calculated from the POV angle
     */
    public double[] convertPOVtoSpeed(double POV) {
        double[] res = new double[2];

        if (POV == -1) {
            res[0] = 0.0;
            res[1] = 0.0;
            return res;
        }
        double radPOV = Math.toRadians(POV);

        res[0] = Math.cos(radPOV);
        res[1] = -Math.sin(radPOV);

        return res;
    }
}
