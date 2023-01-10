package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.common.swerve.FalconConversions;

import static frc.team3128.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Intake subsystem controls rolling wheels of bars, rotation of bars, and conveyer belt in "Star of David" Box.
 */

public class Intake extends SubsystemBase {


    //Motors
    private NAR_TalonSRX m_intakeWheels;

    //Solenoids
    private DoubleSolenoid m_intakeSolenoid;
    
    //Sensors
    private DigitalInput m_intakeSensor;

    private static Intake instance;

    private Intake() {
        configMotors();
        configPneumatics();
        configSensors();
    }

    public static Intake getInstance() {
        if(instance == null){
            instance = new Intake() ;  
        }
        return instance;
    }

    public void configMotors() {
        m_intakeWheels = new NAR_TalonSRX(INTAKE_WHEELS_ID);
    }

    public void configPneumatics() {
        m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                    INTAKE_SOLENOID_FORWARD_CHANNEL_ID, 
                                                    INTAKE_SOLENOID_BACKWARD_CHANNEL_ID);
    }

    public void configSensors() {
        m_intakeSensor = new DigitalInput(SENSOR_INTAKE_ID); //make sure to set sensor_intake_id
    }

    //Solenoid Methods
    public void extendIntake(){
        m_intakeSolenoid.set(Value.kForward);
    }
    public void retractIntake(){
        m_intakeSolenoid.set(Value.kReverse);
    }

    //Power On/Off Method for Wheel Motors and Conveyer Belt Motors
    
    public void enableWheels() {
        m_intakeWheels.set(WHEELS_POWER);
    }

    public void disableWheels() {
        m_intakeWheels.set(0);
    }
    //Sensor Methods
    public boolean getIntakeSensor() {
        return m_intakeSensor.get();
    }
}
