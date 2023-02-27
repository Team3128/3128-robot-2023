package frc.team3128.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;

public class CmdShelfPickup extends SequentialCommandGroup{
    
    private Telescope telescope;

    public CmdShelfPickup (boolean cone, boolean isReversed) {
        telescope = Telescope.getInstance();

        addCommands(
            new InstantCommand(()-> Vision.AUTO_ENABLED = false),
            new CmdMoveLoading(isReversed, VisionConstants.LOADING_ZONE),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            new CmdMoveArm(ArmPosition.HP_SHELF, isReversed), 
            new ParallelDeadlineGroup(
                new CmdManipGrab(cone),
                new RunCommand(()-> Swerve.getInstance().drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.25 : 0.25,0),0,true))
            ),
            new ScheduleCommand(new CmdMoveArm(ArmPosition.NEUTRAL, isReversed)),
            new WaitUntilCommand(()-> telescope.atSetpoint()),
            new ScheduleCommand(new StartEndCommand(() -> RobotContainer.controller.startVibrate(), () -> RobotContainer.controller.stopVibrate()).withTimeout(0.5))
        );
    }
}
