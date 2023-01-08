package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Manipulator;
import static frc.team3128.Constants.ManipulatorConstants.*;

public class CmdReleaseManipulator extends CommandBase {
    private final Manipulator m_manipulator;
    public CmdReleaseManipulator(){
        m_manipulator = Manipulator.getInstance();
        addRequirements(m_manipulator);
        
    }
    @Override
    public void initialize(){
        m_manipulator.openClaw();

    }
    public boolean isFinished() {
        return (m_manipulator.getCurrentTicks() >= MAX_TICKS);
            
        
    }
}
