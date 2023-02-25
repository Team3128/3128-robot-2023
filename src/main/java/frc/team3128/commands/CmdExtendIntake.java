package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;

public class CmdExtendIntake extends SequentialCommandGroup{

    private Manipulator manipulator;
    private Intake intake;

    public CmdExtendIntake(){
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

