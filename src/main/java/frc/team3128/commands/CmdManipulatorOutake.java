package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Manipulator;

public class CmdManipulatorOutake extends WaitCommand{
    private Manipulator manipulator;

    public CmdManipulatorOutake(){
        super(1);
        manipulator = Manipulator.getInstance();
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        super.initialize();
        manipulator.openClaw();
        manipulator.enableRollers(false);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        manipulator.stopRoller();
    }

    @Override
    public void execute() {
        new WaitCommand(1);
        manipulator.stopRoller();
    }
}
