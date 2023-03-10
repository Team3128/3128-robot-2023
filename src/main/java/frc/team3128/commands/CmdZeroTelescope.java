package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Telescope;

public class CmdZeroTelescope extends CommandBase {

    private Telescope m_telescope; 

    @Override
    public void initialize() {
        m_telescope = Telescope.getInstance();
        m_telescope.retract();
    }

    @Override
    public void execute() {
        if (!m_telescope.getLimitSwitch()) {
            m_telescope.stopTele();
            m_telescope.engageBrake();
            m_telescope.zeroEncoder();
        }
    }
}
