package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;
import static frc.team3128.Constants.PivotConstants.*;
import static frc.team3128.Constants.TelescopeConstants.*;

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
            new InstantCommand(() -> telescope.startPID(MIN_DIST), telescope),
            new InstantCommand(() -> pivot.startPID(MIN_ANGLE), pivot)
        );
    }
}
