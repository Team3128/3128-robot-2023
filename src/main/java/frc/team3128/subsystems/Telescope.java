package frc.team3128.subsystems;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.MathUtil;
import static frc.team3128.Constants.TelescopeConstants.*;
import java.util.function.DoubleSupplier;

import frc.team3128.Constants.PivotConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.utility.NAR_Shuffleboard;

import static frc.team3128.Constants.TelescopeConstants;

/**
 * Telescope for windmill arm class
 */

public class Telescope extends PIDSubsystem {

    private DoubleSupplier kG, kF, setpoint;
    
    private static Telescope instance;

    private NAR_CANSparkMax m_teleMotor;
    private SparkMaxRelativeEncoder m_encoder;

    public Telescope() {
        super(new PIDController(kP, kI, kD));

        configMotors();
        configEncoders();
        getController().setTolerance(TELE_TOLERANCE);

        setSetpoint(MIN_DIST);
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
        m_teleMotor.setIdleMode(IdleMode.kBrake);

    }

    private void configEncoders() {
        m_encoder = (SparkMaxRelativeEncoder) m_teleMotor.getEncoder();
        m_encoder.setPositionConversionFactor(ENC_CONV); 
    }

    public void startPID(double teleDist) {
        // super.setSetpoint(setpoint.getAsDouble()); // use for shuffleboard tuning
        enable();
        setSetpoint(teleDist);
        //checkConstraints();
    }

    @Override
    protected void useOutput(double output, double setpoint) {

        double pivotAngle = Math.toRadians(Pivot.getInstance().getMeasurement());
        double ff = -kG.getAsDouble() * Math.cos(pivotAngle) + kF.getAsDouble();
        double voltageOutput = output + ff;
        NAR_Shuffleboard.addData("Testing", "Voltage Output", voltageOutput, 0, 0);

        m_teleMotor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }


    @Override
    protected double getMeasurement() {
       return m_encoder.getPosition() + MIN_DIST;
    }

    public void extend() {
        disable();
        m_teleMotor.set(0.3);
    }

    public void retract() {
        disable();
        m_teleMotor.set(-0.3);
    }

    /**
     * Data for Shuffleboard
     */
    public void initShuffleboard() {
        NAR_Shuffleboard.addData("telescope","telescope dist", ()->getMeasurement(),0,0);
        NAR_Shuffleboard.addData("telescope", "telescope setpoint",()->getSetpoint(), 0, 1);

        kG = NAR_Shuffleboard.debug("telescope","kG", TelescopeConstants.kG,0,2);
        kF = NAR_Shuffleboard.debug("telescope", "kF", TelescopeConstants.kF, 1, 2);
        setpoint = NAR_Shuffleboard.debug("telescope","setpoint", TelescopeConstants.MIN_DIST, 2,0);
        NAR_Shuffleboard.addComplex("telescope", "tele-PID", m_controller, 2, 0);

        NAR_Shuffleboard.addData("telescope", "atSetpoint", ()->getController().atSetpoint(), 3, 0);
        NAR_Shuffleboard.addData("telescope", "isEnabled", ()->isEnabled(), 4, 0);
        
    }

    /**
     * Telescope goes into neutral position (sets power to 0)
     */
    public void resetToDefault() {
        startPID(MIN_DIST);
    }

    public void stopTele() {
        m_teleMotor.set(0);
    }

    public void zeroEncoder() { //returns inches
        m_encoder.setPosition(0);
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

}
