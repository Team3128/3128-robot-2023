package frc.team3128.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import static frc.team3128.Constants.FieldConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.subsystems.Swerve;

public class CmdBalance extends PIDCommand {

    private boolean LEFT;
    // TODO: make it go further than the center (but corresponding to the side that we want to go)

    public CmdBalance () {
        super(
            new PIDController(1.5,0,0),
            () -> Swerve.getInstance().getPose().getX(),
            //() -> DriverStation.getAlliance() == Alliance.Red ? 12.55 : 3.95,
            () -> DriverStation.getAlliance() == Alliance.Red ? FIELD_X_LENGTH - (chargingStationInnerX + chargingStationOuterX) / 2.0 : (chargingStationInnerX + chargingStationOuterX) / 2.0,
            output -> Swerve.getInstance().drive(new Translation2d(0,0), 0,true),
            Swerve.getInstance()
        );
        m_controller.setTolerance(SwerveConstants.DRIVE_TOLERANCE);
    }

    @Override
    public void initialize() {
        super.initialize();
        LEFT = Swerve.getInstance().getPose().getX() > m_setpoint.getAsDouble();
        double ff = LEFT ? -AutoConstants.BALANCE_FF : AutoConstants.BALANCE_FF;
        m_useOutput = output -> Swerve.getInstance().drive(new Translation2d(output + ff,0), 0,true);
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = Swerve.getInstance().getPose();
        return m_controller.atSetpoint() || (pose.getX() < m_setpoint.getAsDouble() && LEFT) || (pose.getX() > m_setpoint.getAsDouble() && !LEFT);
    }

}