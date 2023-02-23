package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Manipulator;
//import static frc.team3128.Constants.ManipulatorConstants.*;

public class CmdManipulatorOutake extends CommandBase{
    private Manipulator manipulator;

    public CmdManipulatorOutake(){
        manipulator = Manipulator.getInstance();
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        super.initialize();
        manipulator.openClaw();
        manipulator.enableRollersReverse();
    }

    @Override
    public void end(boolean interrupted) {
        manipulator.stopRoller();
    }

    // @Override
    // public boolean isFinished() {
    //     return !hasObjectPresent();
    // }

}
