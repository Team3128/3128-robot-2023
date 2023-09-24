package frc.team3128.subsystems;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.MathUtil;
import static frc.team3128.Constants.TelescopeConstants.*;

import java.util.function.DoubleSupplier;
import common.subsystems.NAR_PIDSubsystem;
import common.utility.Controller.PController;

import frc.team3128.RobotContainer;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.Constants.TelescopeConstants;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import frc.team3128.common.utility.NAR_Shuffleboard;

import static frc.team3128.Constants.TelescopeConstants;

/**
 * Telescope for windmill arm
 */

public class Telescope extends NAR_PIDSubsystem {
    
    private static Telescope instance;

    private NAR_CANSparkMax m_teleMotor;
    private DoubleSolenoid m_solenoid; 

    public Telescope() {
        super(new PController(kS, kG, kP, kI, kD));
        
        setkG_Function(()-> {
            if (!atSetpoint()) releaseBrake();

            double pivotAngle = Math.toRadians(Pivot.getInstance().getMeasurement());
            double fG = Math.cos(pivotAngle);
            return fG;
        });

        configMotors();
        configEncoders();
        configPneumatics();
        getController().setTolerance(TELE_TOLERANCE);
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
        m_teleMotor.setCurrentLimit(TELE_CURRENT_LIMIT);
        m_teleMotor.enableVoltageCompensation(12.0);
        m_teleMotor.setNeutralMode(Neutral.BRAKE);

    }

    public void configPneumatics() {
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SOLENOID_FORWARD_CHANNEL_ID, SOLENOID_BACKWARD_CHANNEL_ID);
        engageBrake();
    }

    private void configEncoders() {
        m_teleMotor.setConversionFactor(ENC_CONV);
    }


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
        return -m_teleMotor.getRawPosition() + MIN_DIST;
    }

    @Override
    protected double getMeasurement() {
       return getDist();
    }

    @Override
    protected void useOutput(double output) {
        m_teleMotor.set(MathUtil.clamp(output / 12.0, -1, 1));
    }

    public void releaseBrake() {
        m_solenoid.set(Value.kReverse);
    }

    public void engageBrake() {
        m_solenoid.set(Value.kForward);
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
        m_teleMotor.resetRawPosition(dist);
    }

    public void startPID(ArmPosition position) {
        startPID(position.teleDist);
    }
}
