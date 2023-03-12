package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.common.utility.NAR_Shuffleboard;
import static frc.team3128.Constants.ManipulatorConstants.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
public class V2Manipulator extends SubsystemBase {
    
    private DoubleSolenoid m_solenoid;
    private NAR_TalonSRX m_roller;

    private static V2Manipulator instance;

    public static boolean objectPresent = false;

    public V2Manipulator(){
        configPneumatics();
        configMotor();
    }

    public static V2Manipulator getInstance() {
        if (instance == null){
            instance = new V2Manipulator();  
        }
        
        return instance;
    }

    public void configPneumatics(){
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SOLENOID_FORWARD_CHANNEL_ID, SOLENOID_BACKWARD_CHANNEL_ID);
        // closeClaw();
    }

    public void configMotor(){
        m_roller = new NAR_TalonSRX(ROLLER_MOTOR_ID);
        m_roller.setInverted(false);
        m_roller.setNeutralMode(NeutralMode.Brake);
    }

    public void retractMnp() {
        m_solenoid.set(Value.kReverse);
    }

    public void extendMnp() {
        m_solenoid.set(Value.kForward);
    }
    public void toggleMnp() {
        m_solenoid.toggle();
    }

    public void setRollerPower(double power){
        m_roller.set(power);
    }

    public void enableRollersForward(){
        m_roller.set(ROLLER_POWER);
    }

    public void enableRollersReverse(){
        m_roller.set(-ROLLER_POWER);
    }

    public void stopRoller(){
        m_roller.set(0);
    }

    public double getCurrent(){
        return m_roller.getStatorCurrent();
    }

    public boolean hasObjectPresent(){
        return getCurrent() > CURRENT_THRESHOLD;
    }

    public void intake(boolean cone, boolean shelf) {
        if (cone && shelf) {
            extendMnp();
            enableRollersForward();
        } else if (!cone && shelf) {
            // TODO determine piston state
            enableRollersReverse();
        } else {
            retractMnp();
            enableRollersReverse();
        }

    }    

    public void outtake(boolean cone){
        if (cone) {
            extendMnp();
            enableRollersReverse();
        }
        else {
            retractMnp();
            enableRollersForward();
        }
    }

    public void initShuffleboard() {
        // NAR_Shuffleboard.addData("V2Manipulator","Value", () -> getClawState().toString(),0,0);
        NAR_Shuffleboard.addData("V2Manipulator", "Manip current", () -> getCurrent(), 0, 1);
        // NAR_Shuffleboard.addData("V2Manipulator", "Has object", () -> hasObject(), 0, 2);
        NAR_Shuffleboard.addData("V2Manipulator", "get", () -> m_roller.getMotorOutputPercent(), 0, 3);
        NAR_Shuffleboard.addData("V2Manipulator", "ObjectPresent", ()-> objectPresent, 1, 1);
    }
}
