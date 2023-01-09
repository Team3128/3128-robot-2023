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
    
    private static Telescope instance;

    private NAR_CANSparkMax m_teleMotor;
    private SparkMaxRelativeEncoder m_encoder;
    public double m_ff;
    public double m_setpoint;


    public Telescope() {
        super(new PIDController(kP, kI, kD));

        configMotors();
        configEncoders();
    }

    public synchronized static Telescope getInstance() {
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

        // m_teleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 0); 
        // m_teleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 0);
        // m_teleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0);
        // m_teleMotor.setControlFramePeriodMs(0);

    }

    private void configEncoders() {
        m_encoder = (SparkMaxRelativeEncoder) m_teleMotor.getEncoder();
        m_encoder.setPositionConversionFactor(ENC_CONV);
    }

    public void startPID(double dist) {        
        enable();
        super.setSetpoint(dist);
        getController().setTolerance(TELE_TOLERANCE);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double ff = m_ff * setpoint; //Need to calculate this
        double voltageOutput = output + ff;

        m_teleMotor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }

    @Override
    protected double getMeasurement() {
       return m_teleMotor.getSelectedSensorPosition() + MIN_DIST;
    }

    /**
     * Data for Shuffleboard <-- worry about that later
     */
    public void initShuffleboard() {
    }

    @Override
    public void periodic(){
        super.periodic();
    }

    /**
     * Telescope goes into neutral position (sets power to 0)
     */
    public void stopTele() {
        m_teleMotor.set(0);
    }

    public void zeroEncoder() {
        m_encoder.setPosition(0);
    }

}
