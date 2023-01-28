package frc.team3128.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Swerve;
import frc.team3128.Constants.BalanceConstants;
import frc.team3128.Constants.BalanceConstants.*;

public class CmdDriveUp extends CommandBase{
    private int direction;
    private double power;

    public CmdDriveUp(int direction) {
        this.direction = direction;
    }
    @Override
    public void initialize() {
        power = direction*BalanceConstants.DRIVE_UP_POWER;
    }
    @Override
    public void execute() {
        Swerve.getInstance().drive(new Translation2d(power,0), 0,false);
    }
    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(Swerve.getInstance().getPitch()) >= 10);
    }
}
