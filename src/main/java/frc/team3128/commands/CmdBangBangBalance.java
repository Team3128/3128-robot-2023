package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Swerve;

public class CmdBangBangBalance extends CommandBase{
    public int plateauCount = 0;
    private double power;
    private Swerve swerve;
    private Pose2d pose;
    private double chargeStation;
    private BangBangController controller;
    private double prevRollRate, prevRoll; 
    private static DoubleSupplier thresh, time;

    static {
        thresh = NAR_Shuffleboard.debug("Aflack","Popeyes", 6, 0, 1);
        time = NAR_Shuffleboard.debug("Aflack", "Timer", 0.5, 2, 0);
    }


    public CmdBangBangBalance() {
        
        swerve = Swerve.getInstance();
        power = .4;
        plateauCount = 0;
        prevRollRate = 0;
        prevRoll = swerve.getRoll();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        plateauCount = 0;
        prevRollRate = 0;
        prevRoll = swerve.getRoll();
        var pose = swerve.getPose();
        chargeStation = DriverStation.getAlliance() == Alliance.Red ? 12.6 : 3.85;
        power = pose.getX() > chargeStation ? -0.4 : 0.4;
    }
    @Override
    public void execute() {
        swerve.drive(new Translation2d(power,0), 0,true);
        if (Math.abs(swerve.getRoll()) < thresh.getAsDouble()) 
            plateauCount += 1;
        else {
            plateauCount = 0;
        }
        NAR_Shuffleboard.addData("Aflack", "Geico", swerve.getRoll() - prevRoll/ 0.02,0,0);
        prevRoll = swerve.getRoll();
    }

    @Override
    public void end(boolean interrupted) {
        // Timer.delay(time.getAsDouble());
        new RunCommand(()-> swerve.xlock(), swerve).schedule();
    }

    @Override
    public boolean isFinished() {
        return plateauCount >= 3;
    }
}