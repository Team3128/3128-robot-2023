package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Manipulator;

public class CmdManipulatorMove extends CommandBase {

    private Manipulator m_manipulator;
    private Manipulator.ManipulatorState state;

    public CmdManipulatorMove(Manipulator manipulator, Manipulator.ManipulatorState state) {
        m_manipulator = manipulator;
        this.state = state;
        addRequirements(m_manipulator);
    }

    @Override
    public void initialize() {
        m_manipulator.beginPID(state);
    }

    @Override
    public boolean isFinished() {
        return m_manipulator.isReady();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_manipulator.stop();
    }
}
