package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Led;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.subsystems.Pivot; 
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;

public class CmdShelfPickup extends SequentialCommandGroup{
    
    private Telescope telescope;
    private Pivot pivot;
    private NAR_XboxController controller;
    private Led led;

    public CmdShelfPickup (boolean cone) {
        telescope = Telescope.getInstance();
        pivot = Pivot.getInstance();
        controller = RobotContainer.controller;
        led = Led.getInstance();

        addCommands(
            new InstantCommand(() -> led.setPickupColor(cone)),
            new InstantCommand(()-> Vision.AUTO_ENABLED = false),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            // Commands.parallel(
            //     new CmdMoveLoading(isReversed, VisionConstants.LOADING_ZONE),
            new InstantCommand(() -> pivot.startPID(cone ? ArmPosition.HP_SHELF_CONE : ArmPosition.HP_SHELF_CUBE), pivot),
            new WaitUntilCommand(()-> pivot.atSetpoint()),
            new InstantCommand(() -> telescope.startPID(cone ? ArmPosition.HP_SHELF_CONE : ArmPosition.HP_SHELF_CUBE), telescope),
            new WaitUntilCommand(()-> telescope.atSetpoint()),
            new CmdManipGrab(cone),
            new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> RobotContainer.controller.startVibrate(), () -> RobotContainer.controller.stopVibrate()))),
            new InstantCommand(() -> new InstantCommand(()-> Vision.AUTO_ENABLED = false))
            //new CmdMoveArm(ArmPosition.NEUTRAL)
        );
    }
}
