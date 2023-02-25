package frc.team3128.commands;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Swerve;
import frc.team3128.Constants.BalanceConstants;
import frc.team3128.Constants.BalanceConstants.*;

public class CmdBangBangBalance extends CommandBase{
    private double power;
    private Swerve swerve;
    private Pose2d pose;
    private double chargeStation;
    private BangBangController controller;


    public CmdBangBangBalance() {
        
        swerve = Swerve.getInstance();
        power = 1;

        addRequirements(swerve);
    }


    @Override
    public void execute() {
        swerve.drive(new Translation2d(power,0), 0,true);
    }
    @Override
    public void end(boolean interrupted) {
        swerve.xlock();
    }

    @Override
    public boolean isFinished() {
        return swerve.getRollRate() < 0;
    }
}