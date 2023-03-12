package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Intake;

public class CmdMoveIntake extends CommandBase{

    private double desiredAngle;
    private Intake intake = Intake.getInstance();

    public CmdMoveIntake(double angle) {
        desiredAngle = angle;
        addRequirements(intake);
    }

    public CmdMoveIntake(Intake.IntakeState state) {
        this(state.angle);
    }

    @Override
    public void initialize(){
        intake.startPID(desiredAngle);
    }

    @Override
    public boolean isFinished(){
        return intake.atSetpoint();
    }
}
