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

    public double getFinalSpeed(double baseSpeed, boolean isFastMode, boolean isSlowMode) {
        if (isFastMode) {
            return fastSpeedPCT * baseSpeed;
        } else if (isSlowMode) {
            return slowSpeedPCT * baseSpeed;
        }
        return defaultSpeedPCT * baseSpeed;
    }

    /**
     * 
     * @param POV
     * @param isFastMode
     * @param isSlowMode
     * @return speed in [X, Y]
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
        res[1] = Math.sin(radPOV);

        return res;
    }
}
