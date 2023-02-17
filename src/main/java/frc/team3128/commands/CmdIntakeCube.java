package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;

public class CmdIntakeCube extends SequentialCommandGroup {
    public CmdIntakeCube() {
        var intake = Intake.getInstance();
        var pivot = Pivot.getInstance();
        var manipulator = Manipulator.getInstance();
        addCommands(
            new InstantCommand(()->pivot.startPID(90), pivot),
            new InstantCommand(()-> manipulator.closeClaw()),
            new WaitUntilCommand(()-> pivot.atSetpoint()),
            new InstantCommand(()-> intake.startPID(Intake.IntakeState.DEPLOYED), intake),
            new InstantCommand(()-> intake.enableRollers(0.3), intake)
        );
    }
}
