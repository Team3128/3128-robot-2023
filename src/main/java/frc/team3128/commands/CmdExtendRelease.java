package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class CmdExtendRelease extends SequentialCommandGroup {
    
    private Telescope telescope;
    private Manipulator manipulator;

    public CmdExtendRelease() {
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();
        addCommands(
            new InstantCommand(() -> telescope.startPID(Vision.position), telescope),
            new WaitUntilCommand(()-> telescope.atSetpoint()),
            new WaitCommand(0.125),
            new InstantCommand(() -> manipulator.outtake(), manipulator),
            new WaitCommand(0.125),
            new InstantCommand(() -> manipulator.stopRoller(), manipulator),
            new CmdMoveArm(ArmPosition.NEUTRAL),
            new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> RobotContainer.controller.startVibrate(), () -> RobotContainer.controller.stopVibrate())))
        );
    }
}
