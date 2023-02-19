package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.ArmConstants;
import frc.team3128.subsystems.Telescope;

public class CmdMoveTelescope extends CommandBase{
    
    private double desiredDistance;
    private Telescope telescope = Telescope.getInstance();

    public CmdMoveTelescope(double distance) {
        desiredDistance = distance;
        addRequirements(telescope);
    }

    public CmdMoveTelescope(ArmConstants.ScoringPosition state) {
        desiredDistance = state.getTelescopeDist();
    }

    @Override
    public void initialize(){
        telescope.startPID(desiredDistance);
    }

    @Override
    public void end(boolean interrupted){
        telescope.stopTele();
    }

    @Override
    public boolean isFinished(){
        return telescope.atSetpoint();
    }
}
