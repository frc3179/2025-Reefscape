package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SpeedSettings.DriveSpeedSettings;
import frc.robot.Subsystems.DriveSubsystem;

public class TeleopDrive extends Command{
    final DriveSubsystem m_DriveSubsystem;
    Supplier<Double> xSpeed;
    Supplier<Double> ySpeed;
    Supplier<Double> rot;
    Supplier<Boolean> fieldRelative;

    Supplier<Boolean> isSlowMode;
    Supplier<Boolean> isFastMode;
    Supplier<Double> pov;

    double[] finalSpeeds; // [x, y]
    double finalRot;

    final DriveSpeedSettings m_DriveSpeedSettings;

    public TeleopDrive(
            DriveSubsystem m_DriveSubsystem,
            DriveSpeedSettings m_DriveSpeedSettings,
            Supplier<Double> xSpeed, 
            Supplier<Double> ySpeed, 
            Supplier<Double> rot, 
            Supplier<Boolean> fieldRelative,
            Supplier<Boolean> isSlowMode,
            Supplier<Boolean> isFastMode,
            Supplier<Double> pov
        ){

        this.m_DriveSubsystem = m_DriveSubsystem;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.fieldRelative = fieldRelative;

        this.isSlowMode = isSlowMode;
        this.isFastMode = isFastMode;
        this.pov = pov;

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

        m_DriveSubsystem.drive(finalSpeeds[0], finalSpeeds[1], finalRot, fieldRelative.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
