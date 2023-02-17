package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;

public class CmdCubeIntake extends SequentialCommandGroup {
    public CmdCubeIntake() {
        var intake = Intake.getInstance();
        var pivot = Pivot.getInstance();
        var manipulator = Manipulator.getInstance();
        addCommands(
            new InstantCommand(()-> intake.startPID(180)),
            new WaitUntilCommand(()-> intake.atSetpoint()),
            new InstantCommand(()-> intake.disableRollers()),
            new InstantCommand(()-> pivot.startPID(0), pivot),
            new InstantCommand(()-> manipulator.closeClaw())
        );
    }
}
