package frc.team3128.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class CmdScore extends SequentialCommandGroup {
    
    private Pivot pivot;
    private Telescope telescope;
    private NAR_XboxController controller;

    public CmdScore(boolean isReversed, ArmPosition position, int xpos) {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        controller = RobotContainer.controller;
        addCommands(
            new InstantCommand(() -> NarwhalDashboard.setGridCell(xpos,position.height)),
            new InstantCommand(()-> Vision.AUTO_ENABLED = DriverStation.isAutonomous()),
            Commands.deadline(
                new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
                new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true)
            ),
            Commands.parallel(
                new CmdMoveScore(VisionConstants.RAMP_OVERRIDE[xpos], isReversed, VisionConstants.SCORES_GRID[xpos]),
                new InstantCommand(() -> pivot.startPID(-position.pivotAngle), pivot)
            ),
            Commands.deadline(
                Commands.sequence(
                    new WaitUntilCommand(()-> pivot.atSetpoint()),
                    new InstantCommand(() -> telescope.startPID(position.teleDist), telescope),
                    new WaitUntilCommand(()-> telescope.atSetpoint())                    
                ),
                new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true)
            ),
            new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> RobotContainer.controller.startVibrate(), () -> RobotContainer.controller.stopVibrate()))),
            new InstantCommand(() -> Vision.AUTO_ENABLED = DriverStation.isAutonomous())
        );
    }
}
