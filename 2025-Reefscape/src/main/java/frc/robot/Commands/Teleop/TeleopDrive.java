package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class TeleopDrive extends Command{
    DriveSubsystem m_DriveSubsystem;
    Supplier<Double> xSpeed;
    Supplier<Double> ySpeed;
    Supplier<Double> rot;
    Supplier<Boolean> fieldRelative;

    double slowSpeed;
    Supplier<Boolean> isSlowMode;
    double defaultSpeed;
    double fastSpeed;
    Supplier<Boolean> isFastMode;

    double finalXSpeed;
    double finalYSpeed;
    double finalRot;

    public TeleopDrive(
            DriveSubsystem m_DriveSubsystem,
            Supplier<Double> xSpeed, 
            Supplier<Double> ySpeed, 
            Supplier<Double> rot, 
            Supplier<Boolean> fieldRelative,
            double slowSpeed,
            Supplier<Boolean> isSlowMode,
            double defaultSpeed,
            double fastSpeed,
            Supplier<Boolean> isFastMode
        ){

        this.m_DriveSubsystem = m_DriveSubsystem;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.fieldRelative = fieldRelative;

        this.slowSpeed = slowSpeed;
        this.isSlowMode = isSlowMode;
        this.defaultSpeed = defaultSpeed;
        this.fastSpeed = fastSpeed;
        this.isFastMode = isFastMode;

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(isFastMode.get() == true) {
            finalXSpeed = fastSpeed*xSpeed.get();
            finalYSpeed = fastSpeed*ySpeed.get();
            finalRot = fastSpeed*rot.get();
        } else if (isSlowMode.get() == true) {
            finalXSpeed = slowSpeed*xSpeed.get();
            finalYSpeed = slowSpeed*ySpeed.get();
            finalRot = slowSpeed*rot.get();
        }
        finalXSpeed = defaultSpeed*xSpeed.get();
        finalYSpeed = defaultSpeed*ySpeed.get();
        finalRot = defaultSpeed*rot.get();

        m_DriveSubsystem.drive(finalXSpeed, finalYSpeed, finalRot, fieldRelative.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
