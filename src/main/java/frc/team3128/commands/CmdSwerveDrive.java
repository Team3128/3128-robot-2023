package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.subsystems.Swerve;
import static frc.team3128.Constants.SwerveConstants.*;

public class CmdSwerveDrive extends CommandBase {
    private final Swerve swerve;

    private double rotation;
    private Translation2d translation;
    
    private final boolean fieldRelative;

    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;
    private final DoubleSupplier zAxis;
    private final DoubleSupplier throttle;
    
    // when you call this later use getX getY getZ getThrottle
    public CmdSwerveDrive(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier zAxis, DoubleSupplier throttle, boolean fieldRelative) {
        this.swerve = Swerve.getInstance();
        addRequirements(swerve);

        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.zAxis = zAxis;
        this.throttle = throttle;
        this.fieldRelative = fieldRelative;
    }

    @Override
    public void execute() {
        // deadbands are taken care of in NAR_Joystick
        // TODO: add in slewratelimiter here
        translation = new Translation2d(xAxis.getAsDouble(), yAxis.getAsDouble()).times(throttle.getAsDouble()).times(maxSpeed);
        if (DriverStation.getAlliance() == Alliance.Red || !swerve.fieldRelative) {
            translation = translation.rotateBy(Rotation2d.fromDegrees(90));
        }
        else {
            translation = translation.rotateBy(Rotation2d.fromDegrees(-90));
        }
        
        rotation = zAxis.getAsDouble() * maxAngularVelocity;

        SmartDashboard.putBoolean("fieldOriented",swerve.fieldRelative);
        SmartDashboard.putNumber("yAXIS",yAxis.getAsDouble());
        SmartDashboard.putNumber("xAXIS",xAxis.getAsDouble());
        swerve.drive(translation, rotation, swerve.fieldRelative);

    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}