package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;

public class CmdRunIntake extends CommandBase{
    private final Intake intake;
    public CmdRunIntake() {
        intake = Intake.getInstance();
        addRequirements();
    }

    
    @Override
    public void initialize() {
        intake.enableWheels();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
