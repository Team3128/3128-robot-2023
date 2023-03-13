package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.team3128.Constants.PivotConstants.*;

import java.util.function.DoubleSupplier;

import frc.team3128.RobotContainer;
import frc.team3128.Constants.PivotConstants;
import frc.team3128.Constants.TelescopeConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.utility.NAR_Shuffleboard;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.team3128.common.swerve.CTREConfigs.*;


public class Pivot extends PIDSubsystem {

    private DoubleSupplier kF, setpoint;

    private static Pivot instance;
    private NAR_CANSparkMax m_rotateMotor;
    // private SparkMaxRelativeEncoder m_encoder;
    // public CANCoder m_cancoder;
    private DutyCycleEncoder m_encoder;
    public double offset;

    public Pivot() {
        super(new PIDController(kP, kI, kD));

        //getController().enableContinuousInput(-180, 180);

        configMotors();
        configEncoders();
        getController().setTolerance(PIVOT_TOLERANCE);

        setSetpoint(getMeasurement());
    }

    public static synchronized Pivot getInstance(){
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    private void configMotors() {
        m_rotateMotor = new NAR_CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
        m_rotateMotor.setSmartCurrentLimit(PIVOT_CURRENT_LIMIT);
        m_rotateMotor.setInverted(true);
        m_rotateMotor.enableVoltageCompensation(12.0);
        m_rotateMotor.setIdleMode(IdleMode.kBrake);
    }

    private void configEncoders() {
        m_encoder = new DutyCycleEncoder(ENC_DIO_ID);

        // m_cancoder = new CANCoder(CANCODER_ID, "rio");
        // m_cancoder.configFactoryDefault();
        // m_cancoder.configAllSettings(pivotCancoderConfig());
        // Timer.delay(1.5);
        // m_encoder = (SparkMaxRelativeEncoder) m_rotateMotor.getEncoder();
        // m_encoder.setPositionConversionFactor(ENC_CONV);
        //offset = m_cancoder.getAbsolutePosition();
    }

    public void setPower(double power) {
        disable();
        m_rotateMotor.set(power);
    }

    public void resetToDefault() {
        startPID(0);
    }

    public void changeSetpoint(boolean direction) {
        startPID(getSetpoint() + (direction ? 10.0 / 50.0 : -10.0/50.0));
    }

    public double getAngle(){
        // return MathUtil.inputModulus(-m_cancoder.getAbsolutePosition() - ANGLE_OFFSET, -180, 180);
        return -m_encoder.get() * 360 - ANGLE_OFFSET;
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData("pivot","encoder angle", ()->getAngle(),0,3);
        NAR_Shuffleboard.addData("pivot","pivot angle", ()->getMeasurement(),0,0);
        // NAR_Shuffleboard.addData("pivot","cancoder angle", ()->getAngle(),0,3);
        NAR_Shuffleboard.addData("pivot", "pivot setpoint", ()->getSetpoint(), 0, 1);
        kF = NAR_Shuffleboard.debug("pivot","kF", PivotConstants.kF, 0,2);
        setpoint = NAR_Shuffleboard.debug("pivot", "setpoint", 0, 1,2);
        NAR_Shuffleboard.addComplex("pivot", "Pivot-PID",m_controller, 2, 0);
        NAR_Shuffleboard.addData("pivot", "atSetpoint", ()->getController().atSetpoint(), 3, 0);
        NAR_Shuffleboard.addData("pivot", "isEnabled", ()->isEnabled(), 4, 0);
    }

    public void startPID(double anglePos) {
        anglePos = RobotContainer.DEBUG.getAsBoolean() ? setpoint.getAsDouble() : anglePos;
        anglePos = Math.abs(anglePos) > 135 ? 135 * Math.signum(anglePos) : anglePos;

        enable();
        setSetpoint(anglePos);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double ff = kF.getAsDouble() * Math.sin(Units.degreesToRadians(setpoint)); 
        double teleDist = Telescope.getInstance().getDist();
        ff *= ((teleDist-11.5) / (TelescopeConstants.MAX_DIST - TelescopeConstants.MIN_DIST))*0.5 + 1; 

        double voltageOutput = output + ff;
        
        m_rotateMotor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }

    @Override
    public double getMeasurement() { // returns degrees
        return getAngle();
    }

    public void stopPivot() {
        setPower(0);
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }
    
}
