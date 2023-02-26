package frc.team3128.commands;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Swerve;
import frc.team3128.Constants.BalanceConstants;
import frc.team3128.Constants.BalanceConstants.*;

public class CmdBangBangBalance extends CommandBase{
    public int plateauCount = 0;
    private double power;
    private Swerve swerve;
    private Pose2d pose;
    private double chargeStation;
    private BangBangController controller;


    public CmdBangBangBalance() {
        
        swerve = Swerve.getInstance();
        power = .4;
        plateauCount = 0;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        plateauCount = 0;
    }


    @Override
    public void execute() {
        swerve.drive(new Translation2d(-power*Math.signum(swerve.getRoll()),0), 0,true);
        if (Math.abs(Swerve.getInstance().getRoll()) <= 12) {
            plateauCount += 1;
        } else {
            plateauCount = 0;
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