package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.common.utility.NAR_Shuffleboard;
import static frc.team3128.Constants.ManipulatorConstants.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
public class Manipulator extends SubsystemBase {
    
    private DoubleSolenoid m_solenoid;
    private NAR_TalonSRX m_roller;

    private static Manipulator instance;

    public static boolean objectPresent = false;

    public Manipulator(){
        configPneumatics();
        configMotor();
    }

    public static Manipulator getInstance() {
        if (instance == null){
            instance = new Manipulator();  
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

    public void openClaw(){
        m_solenoid.set(Value.kForward);
    }

    public void closeClaw(){
        m_solenoid.set(Value.kReverse);
    }
    
    public void toggleClaw() {
        m_solenoid.toggle();
    }

    // public Value getClawState() {
    //     return m_solenoid.get();
    // }

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

    public void intake(boolean cone) {
        if (cone) closeClaw();
        else openClaw();
        enableRollersForward(); 
    }    

    public void outtake(boolean cone){
        openClaw();
        if (!cone) enableRollersReverse();
    }

    //forbidden method
    public void shootCubes(){
        setRollerPower(-1);
    }

    public void initShuffleboard() {
        // NAR_Shuffleboard.addData("Manipulator","Value", () -> getClawState().toString(),0,0);
        NAR_Shuffleboard.addData("Manipulator", "Manip current", () -> getCurrent(), 0, 1);
        // NAR_Shuffleboard.addData("Manipulator", "Has object", () -> hasObject(), 0, 2);
        NAR_Shuffleboard.addData("Manipulator", "get", () -> m_roller.getMotorOutputPercent(), 0, 3);
        NAR_Shuffleboard.addData("Manipulator", "ObjectPresent", ()-> objectPresent, 1, 1);
    }
}
