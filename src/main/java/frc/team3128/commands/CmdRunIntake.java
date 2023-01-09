package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Intake;

public class CmdRunIntake extends CommandBase{
    private final Intake intake;
    public CmdRunIntake() {
        intake = Intake.getInstance();
        addRequirements(intake);
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
