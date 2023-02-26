package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;

public class CmdRetractIntake extends SequentialCommandGroup{

    private Manipulator manipulator;
    private Intake intake;

    public CmdRetractIntake(){
        manipulator = Manipulator.getInstance();
        intake = Intake.getInstance();

        addCommands(
            new InstantCommand(()-> intake.disableRollers()),
            new CmdMoveIntake(Intake.IntakeState.STOWED),
            new InstantCommand(()-> manipulator.intake(false), manipulator),
            new CmdMoveArm(ArmPosition.NEUTRAL, false),
            new InstantCommand(()-> intake.enableRollersReverse()),
            new WaitCommand(0.1),
            new InstantCommand(()-> manipulator.setRollerPower(0.1)),
            new InstantCommand(()-> intake.disableRollers())
        );
    }
    
}