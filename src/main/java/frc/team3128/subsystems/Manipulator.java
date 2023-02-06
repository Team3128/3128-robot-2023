package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.utility.NAR_Shuffleboard;

import static frc.team3128.Constants.ManipulatorConstants.*;

public class Manipulator extends SubsystemBase {
    
    private DoubleSolenoid m_solenoid;

    private static Manipulator instance;

    public Manipulator(){
        configPneumatics();
    }

    public static Manipulator getInstance() {
        if (instance == null){
            instance = new Manipulator() ;  
        }
        
        return instance;
    }

    public void configPneumatics(){
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SOLENOID_FORWARD_CHANNEL_ID, SOLENOID_BACKWARD_CHANNEL_ID);
        m_solenoid.set(Value.kReverse);
    }

    public void openClaw(){
        m_solenoid.set(Value.kReverse);
    }

    public void closeClaw(){
        m_solenoid.set(Value.kForward);
    }
    
    public void toggleClaw() {
        m_solenoid.toggle();
    }

    public void disableClaw() {
        m_solenoid.set(Value.kOff);
    }

    public Value getClawState() {
        return m_solenoid.get();
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData("Manipulator","Value", () -> getClawState().toString(),0,0);
    }
}
