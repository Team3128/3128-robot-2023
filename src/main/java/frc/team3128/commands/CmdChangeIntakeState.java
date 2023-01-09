package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Intake.IntakeRotationState;;

public class CmdChangeIntakeState extends CommandBase {

    private Intake m_intake;
    private IntakeRotationState m_rotationState;

    public CmdChangeIntakeState(IntakeRotationState rotationState) {
        m_intake = Intake.getInstance();
        this.m_rotationState = rotationState;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.setRotationSetpoint(m_rotationState);
    }

    @Override
    public boolean isFinished() {
        return m_intake.getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopRotation();
    }
    
}
