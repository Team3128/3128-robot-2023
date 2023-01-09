package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Manipulator;
import static frc.team3128.Constants.ManipulatorConstants.*;

public class CmdCloseConeManipulator extends CommandBase {
    private final Manipulator m_manipulator;
    public CmdCloseConeManipulator(){
        m_manipulator = Manipulator.getInstance();
        addRequirements(m_manipulator);
    }
    @Override
    public void initialize(){
        m_manipulator.closeClaw();
    }
    public boolean isFinished() {
        return (m_manipulator.getCurrentTicks() <= MIN_TICKS_CONE);
    }
}
