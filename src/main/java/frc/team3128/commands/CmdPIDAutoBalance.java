package frc.team3128.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import static frc.team3128.Constants.AutoConstants.*;

import frc.team3128.Constants;
import frc.team3128.subsystems.Swerve;

public class CmdPIDAutoBalance extends CommandBase{
    private final Swerve swerve;
    private boolean onRamp;

    private final PIDController controller = 
        new PIDController(
            Constants.AutoConstants.kP,
            Constants.AutoConstants.kI,
            Constants.AutoConstants.kD);

    public CmdPIDAutoBalance(boolean intialDirection) {
        swerve = Swerve.getInstance();
    }
    
    @Override
    public void initialize() {
        onRamp = false;
    }

    @Override
    public void execute() {
        final Rotation2d pitch = Rotation2d.fromDegrees(swerve.getPitch());
        final Rotation2d roll = Rotation2d.fromDegrees(swerve.getRoll());
        final Rotation2d yaw = Rotation2d.fromDegrees(swerve.getYaw());
        final double advAngle = yaw.getCos() * pitch.getDegrees() + yaw.getSin() * roll.getDegrees();

        if (advAngle > RAMP_THRESHOLD) { 
            onRamp = true;
        }

        if (Math.abs(advAngle) < ANGLE_THRESHOLD && onRamp) {
            swerve.xlock();
            return;
        }

        if (onRamp == true) {
            final double x = controller.calculate(advAngle, 0);
            swerve.drive(new Translation2d(x * (advAngle > 0.0 ? -1.0 : 1.0), 0), 0, false);
        }

        swerve.drive(new Translation2d(DRIVE_SPEED, 0), 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.xlock();
    }

    @Override
    public boolean isFinished() {
        return false;

    }

}
