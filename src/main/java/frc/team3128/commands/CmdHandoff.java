package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdHandoff extends SequentialCommandGroup{

    private Pivot pivot;
    private Telescope telescope;
    private Manipulator manipulator;
    private Intake intake;

    public CmdHandoff(){
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();
        intake = Intake.getInstance();

        addCommands(
            new InstantCommand(()-> intake.enableRollersReverse()),
            new WaitCommand(0.25),
            new InstantCommand(()-> manipulator.closeClaw(), manipulator),
            new InstantCommand(()-> intake.disableRollers())
        );
    }
    
}
