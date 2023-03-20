package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Swerve;

public class CmdBangBangBalance extends CommandBase{
    private int plateauCount = 0;
    private Swerve swerve;
    private double maxRoll; 
    private static DoubleSupplier thresh;

    static {
        thresh = NAR_Shuffleboard.debug("Aflack","Popeyes", 75, 0, 1);
    }
    
    public CmdBangBangBalance() {
        
        swerve = Swerve.getInstance();
        plateauCount = 0;
        maxRoll = 0;
    }

    @Override
    public void initialize() {
        plateauCount = 0;
        maxRoll = Math.abs(swerve.getRoll());
    }

    @Override
    public void execute() {
        if (Math.abs(swerve.getRoll()) > maxRoll) {
            plateauCount = 0;
            maxRoll = Math.abs(swerve.getRoll());
        }
        else {
            plateauCount++;
        }
    }

    @Override
    public boolean isFinished() {
        return plateauCount >= thresh.getAsDouble() || Math.abs(swerve.getRoll()) < 3;
    }
}