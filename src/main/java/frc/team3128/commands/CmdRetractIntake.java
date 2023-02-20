package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdRetractIntake extends SequentialCommandGroup{

    private Pivot pivot;
    private Telescope telescope;
    private Manipulator manipulator;
    private Intake intake;

    public CmdRetractIntake(){
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();
        intake = Intake.getInstance();

        addCommands(
            new InstantCommand(()-> intake.disableRollers()),
            new CmdMoveIntake(Intake.IntakeState.STOWED),
            new InstantCommand(()-> manipulator.openClaw(), manipulator),
            new CmdMoveArm(0, 11.5),
            
            new InstantCommand(()-> intake.enableRollersReverse()),
            new WaitCommand(0.25),
            new InstantCommand(()-> manipulator.closeClaw()),
            new InstantCommand(()-> intake.disableRollers())
        );
    }
    
}