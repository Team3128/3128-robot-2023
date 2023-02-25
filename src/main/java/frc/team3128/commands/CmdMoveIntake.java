package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Intake;

public class CmdMoveIntake extends WaitCommand{

    private double desiredAngle;
    private Intake intake = Intake.getInstance();

    public CmdMoveIntake(double angle) {
        super(0.35);
        desiredAngle = angle;
        addRequirements(intake);
    }

    public CmdMoveIntake(Intake.IntakeState state) {
        this(state.angle);
    }

    @Override
    public void initialize(){
        super.initialize();
        intake.startPID(desiredAngle);
    }

    @Override
    public boolean isFinished(){
        return super.isFinished() || intake.atSetpoint();
    }
}
