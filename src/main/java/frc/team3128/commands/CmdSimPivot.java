package frc.team3128.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdSimPivot extends CommandBase {

    private Pivot pivot = Pivot.getInstance();

    private double angle;
    
    public CmdSimPivot(double angle){
        this.angle = angle;
        addRequirements(pivot);
    }

    @Override
    public void initialize(){
        pivot.startPID(angle);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return pivot.atSetpoint();
    }
}
