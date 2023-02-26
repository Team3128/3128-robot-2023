package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Swerve;

public class CmdDriveUp extends CommandBase{
    private double power;
    private Swerve swerve;
    private Pose2d pose;
    private double chargeStation;


    public CmdDriveUp() {
        
        swerve = Swerve.getInstance();
        pose = swerve.getPose();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            chargeStation = 12.6;
        } else {
            chargeStation = 3.85;
        }
        if (pose.getX() > chargeStation) {
            power = -1;
        } 
        else {
            power = 1;
        }
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(power,0), 0,true);
    }
    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(swerve.getRoll()) >= 5);
    }
}