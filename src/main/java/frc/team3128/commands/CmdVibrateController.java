package frc.team3128.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.hardware.input.NAR_XboxController;


public class CmdVibrateController extends CommandBase{
    NAR_XboxController m_controller;
    public CmdVibrateController(NAR_XboxController controller) {
        m_controller = controller;
    }

    @Override
    public void initialize(){
        m_controller.setRumble(RumbleType.kBothRumble, 0.8);
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){
        m_controller.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
