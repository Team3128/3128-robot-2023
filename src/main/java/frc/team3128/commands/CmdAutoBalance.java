package frc.team3128.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.AutoConstants.*;
import frc.team3128.subsystems.Swerve;

public class CmdAutoBalance extends CommandBase{
    private final Swerve swerve;
    private double prevAngle;
    private boolean onRamp;

    public CmdAutoBalance() {
        swerve = Swerve.getInstance();
    }
    
    @Override
    public void initialize() {
        prevAngle = swerve.getPitch();
        onRamp = false;
    }

    @Override
    public void execute() {
        final Rotation2d pitch = Rotation2d.fromDegrees(swerve.getPitch());
        final Rotation2d roll = Rotation2d.fromDegrees(swerve.getRoll());
        final Rotation2d yaw = Rotation2d.fromDegrees(swerve.getYaw());
        final double advAngle = yaw.getCos() * pitch.getDegrees() + yaw.getSin() * roll.getDegrees();
        final double angleVelocity = (advAngle - prevAngle) / 0.02;
        prevAngle = advAngle;

        if (advAngle > RAMP_THRESHOLD) onRamp = true;

        if (Math.abs(advAngle) < ANGLE_THRESHOLD && onRamp) {
            swerve.xlock();
            return;
        }

        if ((advAngle < 0.0 && angleVelocity > VELOCITY_THRESHOLD) || (advAngle > 0.0 && angleVelocity < VELOCITY_THRESHOLD)) {
            swerve.stop();
            return;
        }

        swerve.drive(new Translation2d(onRamp ? DRIVE_SPEED * (advAngle > 0.0 ? 1.0 : -1.0) : DRIVE_SPEED, 0), 0, false);
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
