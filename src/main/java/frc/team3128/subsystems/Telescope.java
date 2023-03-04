package frc.team3128.subsystems;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.MathUtil;
import static frc.team3128.Constants.TelescopeConstants.*;

import java.util.function.DoubleSupplier;

import frc.team3128.RobotContainer;
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
    private DoubleSolenoid m_solenoid; 

    public Telescope() {
        super(new PIDController(kP, kI, kD));

        configMotors();
        configEncoders();
        configPneumatics();
        getController().setTolerance(TELE_TOLERANCE);

        setSetpoint(getMeasurement());
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

    public void configPneumatics(){
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SOLENOID_FORWARD_CHANNEL_ID, SOLENOID_BACKWARD_CHANNEL_ID);
        engageBrake();
    }

    private void configEncoders() {
        m_encoder = (SparkMaxRelativeEncoder) m_teleMotor.getEncoder();
        m_encoder.setPositionConversionFactor(ENC_CONV); 
    }

    public double getDist() {
        return -m_encoder.getPosition() + MIN_DIST + TELE_OFFSET;
    }

    public void startPID(double teleDist) {
        teleDist = RobotContainer.DEBUG.getAsBoolean() ? setpoint.getAsDouble() : teleDist;

        teleDist = teleDist > 50 ? 50 : teleDist;
        teleDist = teleDist < 11.5 ? 11.5 : teleDist;


        releaseBrake();
        enable();

        setSetpoint(teleDist);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if (!getController().atSetpoint())
            releaseBrake();

        double pivotAngle = Math.toRadians(Pivot.getInstance().getMeasurement());
        double ff = -kG.getAsDouble() * Math.cos(pivotAngle) + kF.getAsDouble();
        double voltageOutput = isReversed ? -(output + ff) : output + ff;

        m_teleMotor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }


    @Override
    protected double getMeasurement() {
       return getDist();
    }

    /*If extends actually extends set isReversed to false,
    if extends retracts, set isReversed to true*/
    public void extend() {
        disable();
        releaseBrake();
        m_teleMotor.set(0.40);
    }

    /*If retracts actually retracts set isReversed to false,
    if retracts extends, set isReversed to true*/
    public void retract() {
        disable();
        releaseBrake();
        m_teleMotor.set(-0.40);
    }

    public void releaseBrake(){
        m_solenoid.set(Value.kReverse);
    }

    public void engageBrake(){
        m_solenoid.set(Value.kForward);
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
