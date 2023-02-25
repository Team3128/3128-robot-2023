package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdMoveArm extends CommandBase{

    private Pivot pivot = Pivot.getInstance();
    private Telescope telescope = Telescope.getInstance();

    private double angle;
    private double dist;
    
    public CmdMoveArm(double angle, double dist){
        this.angle = angle;
        this.dist = dist;

        addRequirements(pivot, telescope);
    }

    public CmdMoveArm(ArmPosition position){
        this(position.pivotAngle, position.teleDist);
    }

    @Override
    public void initialize(){
        if (dist >= telescope.getDist()) 
            pivot.startPID(angle);
        else 
            telescope.startPID(dist);
    }

    @Override
    public void execute(){
        if (pivot.atSetpoint() && telescope.getSetpoint() != dist)
            telescope.startPID(dist);
        if (telescope.atSetpoint() && pivot.getSetpoint() != angle) 
            pivot.startPID(angle);
    }

    @Override
    public void end(boolean interrupted){
        telescope.disable();
        telescope.engageBrake();
    }

    @Override
    public boolean isFinished(){
        return pivot.atSetpoint() && telescope.atSetpoint();
    }

}
