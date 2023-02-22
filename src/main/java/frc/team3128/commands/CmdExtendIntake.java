package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdExtendIntake extends SequentialCommandGroup{

    private Pivot pivot;
    private Telescope telescope;
    private Manipulator manipulator;
    private Intake intake;

    public CmdExtendIntake(){
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();
        intake = Intake.getInstance();

        addCommands(
            new InstantCommand(()-> manipulator.closeClaw(), manipulator),
            new CmdMoveArm(90,11.5),
            new CmdMoveIntake(Intake.IntakeState.DEPLOYED),
            new InstantCommand(()-> intake.enableRollersForward())
        );
    }
    
}

