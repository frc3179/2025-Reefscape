/**
 * A command that handles teleoperated driving of the robot.
 *
 * This command takes input from various suppliers for xSpeed, ySpeed, rotation, field relative mode,
 * slow mode, fast mode, POV, and gyro reset. It calculates the final speeds and rotation based on the input
 * and the drive speed settings. The command then drives the robot using the DriveSubsystem.
 *
 * @param m_DriveSubsystem The DriveSubsystem instance to control the robot's drive system.
 * @param m_DriveSpeedSettings The DriveSpeedSettings instance to configure the drive speed settings.
 * @param xSpeed A supplier for the x-axis speed input.
 * @param ySpeed A supplier for the y-axis speed input.
 * @param
 */
package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SpeedSettings.DriveSpeedSettings;
import frc.robot.Subsystems.Drive.DriveSubsystem;

public class TeleopDrive extends Command{
    private final DriveSubsystem m_DriveSubsystem;
    private Supplier<Double> xSpeed;
    private Supplier<Double> ySpeed;
    private Supplier<Double> rot;
    private Supplier<Boolean> fieldRelative;

    private Supplier<Boolean> isSlowMode;
    private Supplier<Boolean> isFastMode;
    private Supplier<Double> pov;
    Supplier<Boolean> resetGyro;

    private double[] finalSpeeds; // [x, y]
    private double finalRot;

    private final DriveSpeedSettings m_DriveSpeedSettings;

    public TeleopDrive(
            DriveSubsystem m_DriveSubsystem,
            DriveSpeedSettings m_DriveSpeedSettings,
            Supplier<Double> xSpeed, 
            Supplier<Double> ySpeed, 
            Supplier<Double> rot,
            Supplier<Boolean> fieldRelative,
            Supplier<Boolean> isSlowMode,
            Supplier<Boolean> isFastMode,
            Supplier<Double> pov,
            Supplier<Boolean> resetGyro
        ){

        this.m_DriveSubsystem = m_DriveSubsystem;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.fieldRelative = fieldRelative;

        this.isSlowMode = isSlowMode;
        this.isFastMode = isFastMode;
        this.pov = pov;
        this.resetGyro = resetGyro;

        finalSpeeds = new double[2];

        this.m_DriveSpeedSettings = m_DriveSpeedSettings;

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (pov.get() != -1) {
            finalSpeeds = m_DriveSpeedSettings.convertPOVtoSpeed(pov.get());
            finalSpeeds[0] = m_DriveSpeedSettings.getFinalSpeed(finalSpeeds[0], isFastMode.get(), isSlowMode.get());
            finalSpeeds[1] = m_DriveSpeedSettings.getFinalSpeed(finalSpeeds[1], isFastMode.get(), isSlowMode.get());
            finalRot = m_DriveSpeedSettings.getFinalSpeed(rot.get(), isFastMode.get(), isSlowMode.get());
        } else {
            finalSpeeds[0] = m_DriveSpeedSettings.getFinalSpeed(xSpeed.get(), isFastMode.get(), isSlowMode.get());
            finalSpeeds[1] = m_DriveSpeedSettings.getFinalSpeed(ySpeed.get(), isFastMode.get(), isSlowMode.get());
            finalRot = m_DriveSpeedSettings.getFinalSpeed(rot.get(), isFastMode.get(), isSlowMode.get());
        }

        if (finalSpeeds [0] == 0 && finalSpeeds[1] == 0 && finalRot == 0) {
            m_DriveSubsystem.drive(0, 0, 0, fieldRelative.get(), resetGyro.get());
            m_DriveSubsystem.setX();
        } else {
            m_DriveSubsystem.drive(finalSpeeds[0], finalSpeeds[1], finalRot, fieldRelative.get(), resetGyro.get());
        }
        //m_DriveSubsystem.drive(finalSpeeds[0], finalSpeeds[1], finalRot, fieldRelative.get(), resetGyro.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
