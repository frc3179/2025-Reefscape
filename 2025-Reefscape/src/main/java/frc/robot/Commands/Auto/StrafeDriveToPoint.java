package frc.robot.Commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class StrafeDriveToPoint extends Command{
    DriveSubsystem m_DriveSubsystem;
    Supplier<Double> xSpeed;
    Supplier<Double> ySpeed;
    Supplier<Double> rot;
    Supplier<Boolean> fieldRelative;

    Supplier<Double> goalPos;
    Supplier<Double> currentPos;
    double errOffset;
    Supplier<Boolean> interupt;

    double finalXSpeed;
    double finalYSpeed;
    double finalRot;

    public StrafeDriveToPoint(
            DriveSubsystem m_DriveSubsystem,
            Supplier<Double> xSpeed, 
            Supplier<Double> ySpeed, 
            Supplier<Double> rot, 
            Supplier<Boolean> fieldRelative,
            Supplier<Double> goalPos,
            Supplier<Double> currentPos,
            double errOffset,
            Supplier<Boolean> interupt
        ){

        this.m_DriveSubsystem = m_DriveSubsystem;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.fieldRelative = fieldRelative;

        this.currentPos = currentPos;
        this.goalPos = goalPos;
        this.errOffset = errOffset;
        this.interupt = interupt;

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        finalXSpeed = xSpeed.get();
        //TODO: Some PID for to supply finalYSpeed
        finalRot = rot.get();

        m_DriveSubsystem.drive(finalXSpeed, finalYSpeed, finalRot, fieldRelative.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if(
            currentPos.get() <= (goalPos.get() + errOffset) && 
            currentPos.get() >= (goalPos.get() - errOffset)
        ) {
            return true;
        }


        return interupt.get();
    }
}
