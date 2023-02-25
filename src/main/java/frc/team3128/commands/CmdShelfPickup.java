package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;

public class CmdShelfPickup extends SequentialCommandGroup{
    
    private Telescope telescope;

    public CmdShelfPickup (boolean cone) {
        telescope = Telescope.getInstance();

        addCommands(
            new InstantCommand(()-> Vision.AUTO_ENABLED = false),
            new CmdMoveLoading(VisionConstants.LOADING_ZONE),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            Commands.parallel(
                new CmdMoveArm(ArmPosition.HP_SHELF),
                new CmdManipGrab(cone)
            ),
            new ScheduleCommand(new CmdMoveArm(ArmPosition.NEUTRAL)),
            new WaitUntilCommand(()-> telescope.atSetpoint()),
            new ScheduleCommand(new StartEndCommand(() -> RobotContainer.controller.startVibrate(), () -> RobotContainer.controller.stopVibrate()).withTimeout(0.5))
        );
    }
}
