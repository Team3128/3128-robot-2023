package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdMoveArm extends CommandBase{

    private Pivot pivot = Pivot.getInstance();
    private Telescope telescope = Telescope.getInstance();
    private Manipulator manipulator = Manipulator.getInstance();

    private double angle;
    private double dist;
    
    public CmdMoveArm(double angle, double dist, boolean reversed){
        this.angle = reversed ? -angle : angle;
        this.dist = dist;

        addRequirements(pivot, telescope);
    }

    public CmdMoveArm(ArmPosition position, boolean reversed){
        this(position.pivotAngle, position.teleDist, reversed);
    }

    @Override
    public void initialize(){
        if (dist >= telescope.getDist()) 
            pivot.startPID(angle);
        else 
            telescope.startPID(dist);

        // if (!manipulator.hasObjectPresent()) manipulator.stopRoller();
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
        // telescope.disable();
        // telescope.engageBrake();
    }

    @Override
    public boolean isFinished(){
        return pivot.atSetpoint() && telescope.atSetpoint();
    }

}
