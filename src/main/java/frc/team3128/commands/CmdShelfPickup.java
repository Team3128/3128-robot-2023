package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.ArmConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;

public class CmdShelfPickup extends SequentialCommandGroup{
    
    private Pivot pivot;
    private Telescope telescope;
    private Manipulator manipulator;


    public CmdShelfPickup (boolean cone, Pose2d... poses) {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();

        addCommands(
            new InstantCommand(()-> Vision.AUTO_ENABLED = false),
            // new CmdMoveLoading(poses),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            Commands.parallel(
                new CmdMoveArm(ArmPosition.HP_SHELF),
                new CmdManipGrab(cone)
            ),
            new ScheduleCommand(new CmdMoveArm(ArmPosition.NEUTRAL)),
            new WaitUntilCommand(()-> telescope.atSetpoint())
        );
    }
}
