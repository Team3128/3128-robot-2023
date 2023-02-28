package frc.team3128.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team3128.subsystems.Swerve;

public class CmdBangBangBalance extends CommandBase{
    public int plateauCount = 0;
    private double power;
    private Swerve swerve;
    private Pose2d pose;
    private double chargeStation;
    private BangBangController controller;
    private double prevRollRate; 


    public CmdBangBangBalance() {
        
        swerve = Swerve.getInstance();
        power = .4;
        plateauCount = 0;
        prevRollRate = swerve.getRollRate();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        plateauCount = 0;
        prevRollRate = swerve.getRollRate();
    }


    @Override
    public void execute() {
        swerve.drive(new Translation2d(-power*Math.signum(swerve.getRoll()),0), 0,true);
        if ((Math.signum(swerve.getRollRate()) != Math.signum(prevRollRate)) && Math.abs(swerve.getRollRate()) > 0.5 && Math.abs(swerve.getRoll()) > 9) 
            plateauCount += 1;
        else {
            plateauCount = 0;
            prevRollRate = swerve.getRollRate();
        }
    }

    @Override
    public void end(boolean interrupted) {
        new RunCommand(()-> swerve.xlock(), swerve).schedule();
    }

    @Override
    public boolean isFinished() {
        return plateauCount >= 3;
    }
}