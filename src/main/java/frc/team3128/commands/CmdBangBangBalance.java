package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Swerve;

public class CmdBangBangBalance extends CommandBase{
    private int plateauCount = 0;
    private Swerve swerve;
    private double maxPitch; 
    private static DoubleSupplier thresh;
    private double pitchOffset;

    static {
        thresh = NAR_Shuffleboard.debug("Aflack","Popeyes", 85, 0, 1);
    }
    
    public CmdBangBangBalance() {
        
        swerve = Swerve.getInstance();
        plateauCount = 0;
        maxPitch = 0;
        pitchOffset = 0;
    }

    @Override
    public void initialize() {
        plateauCount = 0;
        maxPitch = Math.abs(swerve.getPitch());
        pitchOffset = swerve.getPitch();
    }

    @Override
    public void execute() {
        if (Math.abs(swerve.getPitch()) > maxPitch) {
            plateauCount = 0;
            maxPitch = Math.abs(swerve.getPitch());
        }
        else {
            plateauCount++;
        }
    }

    @Override
    public boolean isFinished() {
        return plateauCount >= thresh.getAsDouble() && Math.abs(swerve.getPitch() - pitchOffset) < 9;
    }
}