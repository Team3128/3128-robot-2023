package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.ArmConstants;
import frc.team3128.subsystems.Pivot;

public class CmdMovePivot extends CommandBase{
    
    private double desiredAngle;
    private static Pivot pivot = Pivot.getInstance();

    public CmdMovePivot(double angle) {
        desiredAngle = angle;
        addRequirements(pivot);
    }

    public CmdMovePivot(ArmConstants.ScoringPosition state) {
        desiredAngle = state.getPivotAngle();
    }

    @Override
    public void initialize(){
        pivot.startPID(desiredAngle);
    }

    @Override
    public void end(boolean interrupted){
        pivot.stopPivot();
    }

    @Override
    public boolean isFinished(){
        return pivot.atSetpoint();
    }
}
