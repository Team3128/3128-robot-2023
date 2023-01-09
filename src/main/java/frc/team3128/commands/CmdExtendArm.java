package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdExtendArm extends ParallelCommandGroup {

    private Pivot pivot;
    private Telescope telescope;


    public CmdExtendArm(double angle, double dist) {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();

        addCommands(
           new InstantCommand(() -> pivot.startPID(angle), pivot), 
           new InstantCommand(() -> telescope.startPID(dist), telescope)
        );
    }
}
