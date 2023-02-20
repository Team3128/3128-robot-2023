package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.ArmConstants;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Pivot.*;
import frc.team3128.subsystems.Telescope.*;

/*
 * Moves arm to idle position
 */
public class CmdRetractArm extends SequentialCommandGroup{
   
    private Pivot pivot;
    private Telescope telescope;
    private Manipulator manipulator;

    public CmdRetractArm () {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();

        addCommands(
            new InstantCommand(()-> manipulator.closeClaw()),
            new InstantCommand(() -> telescope.startPID(ArmConstants.ScoringPosition.NEUTRAL.teleDist), telescope),
            new WaitUntilCommand(()-> telescope.atSetpoint()),
            new InstantCommand(() -> pivot.startPID(ArmConstants.ScoringPosition.NEUTRAL.pivotAngle), pivot)
            // new InstantCommand(() -> pivot.startPID(angle), pivot),
            // new WaitCommand(3),
            // new InstantCommand(() -> telescope.startPID(dist), telescope)

        );
    }
}
