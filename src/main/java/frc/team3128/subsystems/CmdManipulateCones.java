package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdManipulateCones extends CommandBase{

    private Manipulator manipulator;

    public CmdManipulateCones() {
        manipulator = Manipulator.getInstance();
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        manipulator.closeClaw();
        manipulator.enableRollersForward();
    }

    @Override
    public void end(boolean interrupted) {
        manipulator.stopRoller();
    }

    // @Override
    // public boolean isFinished(){
    //     return manipulator.hasObjectPresent();
    // }
    
}
