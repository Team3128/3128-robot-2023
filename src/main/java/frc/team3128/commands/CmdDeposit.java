package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;

public class CmdDeposit extends CommandBase{
    private final Intake intake;
    private final Hopper hopper;

    public CmdDeposit() {
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
        addRequirements(intake, hopper);
    }

    @Override
    public void initialize() {
        intake.enableWheels();
        hopper.enableSerializer();
    }

    @Override
    public boolean isFinished() {
        //TODO Add isfinished when beam break code is in hopper subsystem 
        return false;
    }

}
