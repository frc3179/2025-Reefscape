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
}
