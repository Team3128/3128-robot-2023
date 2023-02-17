package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.subsystems.Intake;

public class CmdIntakeCone extends SequentialCommandGroup {
    private Intake intake = Intake.getInstance();

    public CmdIntakeCone() {

        addCommands(
            new InstantCommand(() -> intake.enableRollers(-0.6), intake),
            new WaitUntilCommand(() -> Math.abs(intake.getCurrent()) >= 45),
            new WaitCommand(0.75),
            new InstantCommand(() -> intake.enableRollers(-0.3), intake)
        );
    }
}
