package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Intake.IntakeState;

public class CmdIntakeCube extends SequentialCommandGroup {
    public CmdIntakeCube() {
        var intake = Intake.getInstance();
        var pivot = Pivot.getInstance();
        var telescope = Telescope.getInstance();
        var manipulator = Manipulator.getInstance();
        addCommands(
            new InstantCommand(()-> manipulator.closeClaw()),
            new CmdMovePivot(90),
            new CmdMoveIntake(IntakeState.DEPLOYED),
            new WaitUntilCommand(()-> intake.intakeCube()),
            new CmdMoveIntake(IntakeState.STOWED),
            new InstantCommand(()-> manipulator.openClaw()),
            new CmdMovePivot(0),
            new InstantCommand(()-> intake.enableRollersReverse()),
            new CmdMoveTelescope(2) /* change later */,
            new InstantCommand(()-> manipulator.closeClaw()),
            new CmdMoveTelescope(0),
            new InstantCommand(()-> intake.disableRollers())
        );
    }
}