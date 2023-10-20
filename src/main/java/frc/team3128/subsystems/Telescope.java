package frc.team3128.subsystems;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.MathUtil;
import static frc.team3128.Constants.TelescopeConstants.*;

import java.util.function.DoubleSupplier;

import frc.team3128.RobotContainer;
import frc.team3128.Constants.ArmConstants.ArmPosition;
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
    //private SparkMaxRelativeEncoder m_encoder;
    private Solenoid m_solenoid; 
    //private DigitalInput m_limitSwitch;

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
        m_teleMotor = new NAR_CANSparkMax(TELE_MOTOR_ID);
        m_teleMotor.setSmartCurrentLimit(TELE_CURRENT_LIMIT);
        m_teleMotor.enableVoltageCompensation(12.0);
        m_teleMotor.setIdleMode(IdleMode.kBrake);

    }

    public void configPneumatics(){
        m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        engageBrake();
    }

    private void configEncoders() {
        m_teleMotor.setPositionConversionFactor(ENC_CONV);
        // m_encoder = (SparkMaxRelativeEncoder) m_teleMotor.getEncoder();
        // m_encoder.setPositionConversionFactor(ENC_CONV); 
        // m_limitSwitch = new DigitalInput(9);
    }

    // public boolean getLimitSwitch() {
    //     return m_limitSwitch.get();
    // }

    /*If extends actually extends set isReversed to false,
    if extends retracts, set isReversed to true*/
    public void extend() {
        setPower(-0.4);
    }

    /*If retracts actually retracts set isReversed to false,
    if retracts extends, set isReversed to true*/
    public void retract() {
        setPower(0.4);
    }
    
    public void setPower(double power) {
        disable();
        releaseBrake();
        m_teleMotor.set(power);
    }

    public double getDist() {
        return -m_teleMotor.getSelectedSensorPosition() + MIN_DIST;
    }

    @Override
    protected double getMeasurement() {
       return getDist();
    }

    public void startPID(double teleDist) {
        teleDist = RobotContainer.DEBUG.getAsBoolean() ? setpoint.getAsDouble() : teleDist;
        teleDist = MathUtil.clamp(teleDist,MIN_DIST,MAX_DIST);

        releaseBrake();
        enable();

        setSetpoint(teleDist);
    }

    public void startPID(ArmPosition position) {
        startPID(position.teleDist);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if (!atSetpoint()) releaseBrake();

        double pivotAngle = Math.toRadians(Pivot.getInstance().getMeasurement());
        double ff = -kG.getAsDouble() * Math.cos(pivotAngle) + kF.getAsDouble();
        double voltageOutput = isReversed ? -(output + ff) : output + ff;

        m_teleMotor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }

    public void releaseBrake(){
        m_solenoid.set(false);
    }

    public void engageBrake(){
        m_solenoid.set(true);
    }

    /**
     * Telescope goes into neutral position (sets power to 0)
     */
    public void resetToDefault() {
        startPID(MIN_DIST);
    }

    public void stopTele() {
        disable();
        m_teleMotor.set(0);
        engageBrake();
    }

    public void zeroEncoder() { //returns inches
        zeroEncoder(0);
    }

    public void zeroEncoder(double dist) { //returns inches
        m_teleMotor.setSelectedSensorPosition(dist);
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    /**
     * Data for Shuffleboard
     */
    public void initShuffleboard() {
        NAR_Shuffleboard.addData("telescope","telescope dist", ()->getMeasurement(),0,0);
        NAR_Shuffleboard.addData("telescope", "telescope setpoint",()->getSetpoint(), 0, 1);

        kG = NAR_Shuffleboard.debug("telescope","kG", TelescopeConstants.kG,0,2);
        kF = NAR_Shuffleboard.debug("telescope", "kF", TelescopeConstants.kF, 1, 2);
        setpoint = NAR_Shuffleboard.debug("telescope","setpoint", TelescopeConstants.MIN_DIST, 3,1);
        NAR_Shuffleboard.addComplex("telescope", "tele-PID", m_controller, 2, 0);

        NAR_Shuffleboard.addData("telescope", "atSetpoint", ()->getController().atSetpoint(), 3, 0);
        NAR_Shuffleboard.addData("telescope", "isEnabled", ()->isEnabled(), 4, 0);

        // NAR_Shuffleboard.addData("telescope", "limit switch",()-> getLimitSwitch(), 5, 0);   
    }

}
