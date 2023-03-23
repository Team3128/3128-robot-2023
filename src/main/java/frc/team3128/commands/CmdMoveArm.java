package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdMoveArm extends CommandBase{

    private Pivot pivot = Pivot.getInstance();
    private Telescope telescope = Telescope.getInstance();

    private double angle;
    private double dist;
    private boolean teleStatic;
    private Boolean cone;
    
    public CmdMoveArm(double angle, double dist){
        this.angle = angle;
        this.dist = dist;

        addRequirements(pivot, telescope);
    }

    public CmdMoveArm(ArmPosition position){
        this.angle = position.pivotAngle;
        this.dist = position.teleDist;

        if (position.cone == null) cone = null;
        else cone = position.cone;
        
        addRequirements(pivot, telescope);
    }

    @Override
    public void initialize(){

        teleStatic = false;
        // if (cone != null) Manipulator.CONE = cone;

        if (dist >= telescope.getDist()) {
            pivot.startPID(angle);
            if (dist == telescope.getSetpoint()) teleStatic = true;
        }
        else telescope.startPID(dist);

    }

    @Override
    public void execute(){
        if (pivot.atSetpoint() && (telescope.getSetpoint() != dist) && !teleStatic)
            telescope.startPID(dist);
        if (telescope.atSetpoint() && (pivot.getSetpoint() != angle || !pivot.isEnabled())) 
            pivot.startPID(angle);
    }

    @Override
    public void end(boolean interrupted){
        telescope.stopTele(); 
    }

    @Override
    public boolean isFinished(){
        return pivot.atSetpoint() && (telescope.atSetpoint() || teleStatic);
    }

}
