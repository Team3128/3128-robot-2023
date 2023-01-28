package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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


    public CmdRetractArm () {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();

        addCommands(
            new InstantCommand(() -> telescope.startPID(TeleDists.NEUTRAL.dist), telescope),
            new InstantCommand(() -> pivot.startPID(PivotAngles.NEUTRAL.angle), pivot)
        );
    }
}
