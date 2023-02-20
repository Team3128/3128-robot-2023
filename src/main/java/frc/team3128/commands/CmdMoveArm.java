package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdMoveArm extends CommandBase{

    private static Pivot pivot = Pivot.getInstance();
    private static Telescope telescope = Telescope.getInstance();

    private double angle;
    private double dist;

    private double currAngle;
    private double currDist;
    
    public CmdMoveArm(double angle, double dist){
        this.angle = angle;
        this.dist = dist;

        addRequirements(
            pivot,
            telescope
        );
    }

    @Override
    public void initialize(){
        currAngle = pivot.getAngle();
        currDist = telescope.getDist();
    }

    @Override
    public void execute(){
        if(dist >= currDist){
            pivot.startPID(angle);
            if(pivot.atSetpoint()) telescope.startPID(dist);
        }
        else{
            telescope.startPID(dist);
            if(telescope.atSetpoint()) pivot.startPID(angle);
        }
    }

    @Override
    public void end(boolean interrupted){
        telescope.engageBrake();
    }

    @Override
    public boolean isFinished(){
        return pivot.atSetpoint() && telescope.atSetpoint();
    }

}
