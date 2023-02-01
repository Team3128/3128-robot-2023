package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import static frc.team3128.Constants.TelescopeConstants.*;
import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.*;

import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.utility.NAR_Shuffleboard;

import static frc.team3128.Constants.TelescopeConstants;

/**
 * Telescope for windmill arm class
 */

public class Telescope extends PIDSubsystem {

    public enum TeleDists {
        TOP_CONE(56.75), 
        TOP_CUBE(56.75), 
        MID_CONE(40.027), 
        MID_CUBE(39.031), 
        LOW_FLOOR(16),
        HP_PICK_UP(0.0), //get value from mech
        INT_PICK_UP(0.0), //get value from mech
        NEUTRAL(16);

        public double dist; 
        private TeleDists(double dist) {
            this.dist = dist;
        }
    }
    
    private static Telescope instance;

    private NAR_CANSparkMax m_teleMotor;
    private SparkMaxRelativeEncoder m_encoder;

    public Telescope() {
        super(new PIDController(kP, kI, kD));

        configMotors();
        configEncoders();
    }

    public static synchronized Telescope getInstance() {
        if (instance == null)
            instance = new Telescope();
        return instance;
    }
    
    /**
     * Initializes motor needed for telelscope and sets up CAN frame periods
     */
    private void configMotors() {
        m_teleMotor = new NAR_CANSparkMax(TELE_MOTOR_ID, MotorType.kBrushless);
        m_teleMotor.setSmartCurrentLimit(TELE_CURRENT_LIMIT);
        m_teleMotor.enableVoltageCompensation(12.0);
        m_teleMotor.setIdleMode(IdleMode.kCoast);
    }

    private void configEncoders() {
        m_encoder = (SparkMaxRelativeEncoder) m_teleMotor.getEncoder();
        m_encoder.setPositionConversionFactor(ENC_CONV); // TODO: ticks --> inches using gear ratio
    }

    public void startPID(TeleDists teleDist) {        
        enable();
        super.setSetpoint(teleDist.dist);
        getController().setTolerance(TELE_TOLERANCE);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double ff = kF * setpoint; //Need to calculate this
        double voltageOutput = output + ff;
        // TODO: account for rotation in pivot b/c gravity

        m_teleMotor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }

    @Override
    protected double getMeasurement() {
       return m_teleMotor.getSelectedSensorPosition() + MIN_DIST;
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData("Pivot + Tele", "Tele Angle", () -> getMeasurement(), 0, 0);
        NAR_Shuffleboard.addComplex("Pivot + Tele", "Tele PID", getController(), 0, 1);
    }


    /**
     * Telescope goes into neutral position (sets power to 0)
     */
    public void stopTele() {
        disable();
    }

    public void zeroEncoder() {
        m_teleMotor.setEncoderPosition(0);
    }

}
