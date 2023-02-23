package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Manipulator;

public class CmdManipulatorOutake extends CommandBase{
    private Manipulator manipulator;

    public CmdManipulatorOutake(){
        manipulator = Manipulator.getInstance();
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        manipulator.openClaw();
        manipulator.enableRollers(false);
    }

    @Override
    public void execute() {
        new WaitCommand(1);
        manipulator.stopRoller();
    }
}
