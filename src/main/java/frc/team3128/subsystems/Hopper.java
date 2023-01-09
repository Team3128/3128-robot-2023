package frc.team3128.subsystems;

import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import static frc.team3128.Constants.HopperConstants.*;

public class Hopper {
    
    private NAR_TalonSRX m_serializer;

    private static Hopper instance;

    private Hopper() {
       configMotors();
    }

    private void configMotors() {
        m_serializer = new NAR_TalonSRX(SERIALIZER_ID);
    }

    public static Hopper getInstance() {
        if(instance == null){
            instance = new Hopper() ;  
        }
        return instance;
    }

    public void enableSerializer() {
        m_serializer.set(SERIALIZER_POWER);
    }

    public void disableSerializer() {
        m_serializer.set(0);
    }

}
