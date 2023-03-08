package frc.team3128.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import static frc.team3128.Constants.FieldConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Swerve;

public class CmdBalance extends PIDCommand {

    public CmdBalance () {
        super(
            new PIDController(0.5,0,0),
            () -> Swerve.getInstance().getPose().getX(),
            () -> DriverStation.getAlliance() == Alliance.Red ? FIELD_X_LENGTH - (chargingStationInnerX + chargingStationOuterX) / 2.0 : (chargingStationInnerX + chargingStationOuterX) / 2.0,
            output -> Swerve.getInstance().drive(new Translation2d(output + Math.copySign(AutoConstants.BALANCE_FF, output),0), 0,true),
            Swerve.getInstance()
        );
        m_controller.setTolerance(SwerveConstants.DRIVE_TOLERANCE);
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}