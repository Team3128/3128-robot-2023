package frc.team3128;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdExtendArm extends SequentialCommandGroup{
    private double angle;
    private double dist;

    private Pivot pivot = Pivot.getInstance();
    private Telescope telescope = Telescope.getInstance();

    public CmdExtendArm(double angle, double dist) {
        this.angle = angle;
        this.dist = dist;

        addCommands(
            new InstantCommand(()-> telescope.startPID(11.5)),
            new WaitUntilCommand(()-> telescope.atSetpoint()),
            new InstantCommand(()-> pivot.startPID(angle)),
            new WaitUntilCommand(()-> pivot.atSetpoint()),
            new InstantCommand(()-> telescope.startPID(dist)),
            new WaitUntilCommand(() -> telescope.atSetpoint())
        );
    }


}
