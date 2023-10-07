package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Swerve;
import static frc.team3128.Constants.SwerveConstants.*;

public class CmdSwerveDrive extends CommandBase {
    private final Swerve swerve;

    private double rotation;
    private Translation2d translation;

    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;
    private final DoubleSupplier zAxis;

    private final SlewRateLimiter accelLimiter;

    private final PIDController rController;
    public static boolean enabled = false;
    public static double rSetpoint;
    
    public CmdSwerveDrive(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier zAxis, boolean fieldRelative) {
        this.swerve = Swerve.getInstance();
        addRequirements(swerve);

        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.zAxis = zAxis;

        accelLimiter = new SlewRateLimiter(maxAcceleration);
        rController = new PIDController(turnKP, turnKI, turnKD);
        rController.enableContinuousInput(0, 360);
        rController.setTolerance(0.5);
        NAR_Shuffleboard.addComplex("Drivetrain", "rController", rController, 5, 4);
        swerve.fieldRelative = fieldRelative;
    }

    @Override
    public void execute() {
        // deadbands are taken care of in NAR_Joystick
        // TODO: add in slewratelimiter here
        translation = new Translation2d(xAxis.getAsDouble(), yAxis.getAsDouble()).times(Swerve.throttle).times(maxSpeed);
        if (DriverStation.getAlliance() == Alliance.Red || !swerve.fieldRelative) {
            translation = translation.rotateBy(Rotation2d.fromDegrees(90));
        }
        else {
            translation = translation.rotateBy(Rotation2d.fromDegrees(-90));
        }
        
        rotation = -zAxis.getAsDouble() * maxAngularVelocity * Swerve.throttle; 
        if (Math.abs(rotation) > 0.25) {
            enabled = false;
        }
        if (enabled) {
            rotation = Units.degreesToRadians(rController.calculate(swerve.getGyroRotation2d().getDegrees(), rSetpoint));
            if (rController.atSetpoint()) {
                rotation = 0;
            }
        }

        Rotation2d driveAngle = translation.getAngle();
        double slowedDist = accelLimiter.calculate(translation.getNorm());
        // translation = new Translation2d(slowedDist, driveAngle);

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