package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.team3128.Constants.PivotConstants.*;

import java.util.function.DoubleSupplier;

import frc.team3128.RobotContainer;
import frc.team3128.Constants.PivotConstants;
import frc.team3128.Constants.TelescopeConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.common.utility.NAR_Shuffleboard;

import com.revrobotics.CANSparkMax.IdleMode;

import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.subsystems.NAR_PIDSubsystem;
import common.utility.Controller.PController;

/**
 * Pivot for windmill arm
 */

public class Pivot extends NAR_PIDSubsystem {

    private static Pivot instance;
    private NAR_CANSparkMax m_rotateMotor;
    public double offset;

    private Pivot() {
        super(new PController(kS, kG, kP, kI, kD));

        setkG_Function(()-> {
            double fG = Math.sin(Units.degreesToRadians(getSetpoint())); 
            double teleDist = Telescope.getInstance().getDist();
            fG *= 1.0/14.25 * (teleDist - TelescopeConstants.MIN_DIST) + 1;
            
            return fG;
        });
        
        configMotors();
        getController().setTolerance(PIVOT_TOLERANCE);
    }

    public static synchronized Pivot getInstance(){
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    private void configMotors() {
        m_rotateMotor = new NAR_CANSparkMax(PIVOT_MOTOR_ID);
        m_rotateMotor.setCurrentLimit(PIVOT_CURRENT_LIMIT);
        m_rotateMotor.setInverted(false);
        m_rotateMotor.setNeutralMode(Neutral.BRAKE);
        resetPivot();
    }

    public void setPower(double power) {
        disable();
        m_rotateMotor.set(power);
    }

    public void resetPivot() {
        m_rotateMotor.resetRawPosition(0);
    }

    public void stopPivot() {
        setPower(0);
    }

    @Override
    protected double getMeasurement() { // returns degrees
        return m_rotateMotor.getRawPosition() * 360 / GEAR_RATIO;
    }

    public void startPID(ArmPosition position) {
        startPID(position.pivotAngle);
    }

    @Override
    protected void useOutput(double output) {
        m_rotateMotor.set(MathUtil.clamp(output / 12.0, -1, 1));
    }

}
