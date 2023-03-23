package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Intake.IntakeState;

public class CmdIntake extends SequentialCommandGroup{

    public CmdIntake() {
        Intake intake = Intake.getInstance();
        
        addCommands(
            new InstantCommand(()-> Intake.objectPresent = false),
            new InstantCommand(()-> intake.intake(), intake),
            new InstantCommand(()-> intake.startPID(IntakeState.DEPLOYED.angle)),
            new WaitUntilCommand(()-> intake.atSetpoint()),
            new WaitCommand(0.1),
            new WaitUntilCommand(()->intake.hasObjectPresent()),
            new InstantCommand(()-> Intake.objectPresent = true),
            new InstantCommand(()->intake.set(IntakeConstants.STALL_POWER), intake),
            new InstantCommand(()-> intake.startPID(IntakeState.RETRACTED.angle))
            // new WaitUntilCommand(()-> intake.atSetpoint())
        );
    }
}
