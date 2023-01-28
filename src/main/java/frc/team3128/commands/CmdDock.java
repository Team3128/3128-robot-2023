package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Swerve;
import frc.team3128.Constants.BalanceConstants;
import frc.team3128.Constants.BalanceConstants.*;

public class CmdDock extends SequentialCommandGroup{
    private Swerve swerve;
    private Pose2d pose2d;
    private double x;
    private double y;
    private int direction;

    public  CmdDock() {
        this.swerve = Swerve.getInstance();
        this.pose2d = swerve.getPose();
        this.x = pose2d.getX();
        this.y = pose2d.getY();

        if (x < BalanceConstants.CHARGE_STATION_X) {
            direction = -1;
        } 
        else {
            direction = 1;
        }


        addCommands(
            new CmdDriveUp(direction),
            new CmdGyroBalance(direction)
        );
    }
    
}
