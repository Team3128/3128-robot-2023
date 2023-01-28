package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.team3128.Constants.PivotConstants.*;

import javax.print.attribute.standard.MediaSize.NA;

import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.utility.NAR_Shuffleboard;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Pivot extends PIDSubsystem {

    public enum PivotAngles {
        TOP_CONE(180 - 81.666), 
        TOP_CUBE(180 - 92.221), 
        MID_CONE(180 - 95.559), 
        MID_CUBE(180 - 110.041), 
        LOW_FLOOR(180 - 155.114), //need new value from mech
        HP_PICK_UP(0.0), //get value from mech
        INT_PICK_UP(0.0), //get value from mech
        NEUTRAL(0.0);

        public double angle; 
        private PivotAngles(double angle) {
            this.angle = angle;
        }
    }

    private static Pivot instance;
    private NAR_CANSparkMax m_rotateMotor;
    private SparkMaxRelativeEncoder m_encoder;

    public Pivot() {
        super(new PIDController(kP, kI, kD));

        configMotors();
        configEncoders();
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
        m_rotateMotor.enableVoltageCompensation(12.0);
        m_rotateMotor.setIdleMode(IdleMode.kBrake);


        // m_rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 0); 
        // m_rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 0);
        // m_rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0);
        // m_rotateMotor.setControlFramePeriodMs(0);
    }

    private void configEncoders() {
        m_encoder = (SparkMaxRelativeEncoder) m_rotateMotor.getEncoder();
        m_encoder.setPositionConversionFactor(ENC_CONV);
    }

    public void stopPivot() {
        disable();
    }

    public void zeroEncoder() {
        // m_rotateMotor.setEncoderPosition(0)
        m_encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        NAR_Shuffleboard.addData("pivot","pivot angle", getMeasurement(),0,0);
        NAR_Shuffleboard.addData("pivot", "pivot setpoint", getSetpoint(), 0, 1);
        // NAR_Shuffleboard.addData("pivot", "")
        super.periodic();
    }

    public void startPID(PivotAngles anglePos) {        
        enable();
        super.setSetpoint(anglePos.angle);
        getController().setTolerance(PIVOT_TOLERANCE);
    }

    public void startPID(double anglePos) {        
        enable();
        super.setSetpoint(anglePos);
        getController().setTolerance(PIVOT_TOLERANCE);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double ff = kF * Math.sin(Units.degreesToRadians(setpoint)); //Need to calculate this
        double voltageOutput = output + ff;

        m_rotateMotor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }

    @Override
    protected double getMeasurement() { // returns degrees
       return m_rotateMotor.getSelectedSensorPosition() + MIN_ANGLE;
    }

    
}
