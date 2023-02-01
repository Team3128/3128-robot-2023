package frc.team3128.subsystems;

import java.util.List;

import javax.print.DocFlavor.STRING;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.LedConstants;
import frc.team3128.Constants.LedConstants.*;

public class Led extends SubsystemBase{

    private static Led instance;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public Led(){
        InitLeds();
    }
    
    public static synchronized Led getInstance() {
        if (instance == null) {
            instance = new Led();
        }
        return instance;
    }
    
    public void InitLeds() {
        m_led = new AddressableLED(LedConstants.PORT);

        m_ledBuffer = new AddressableLEDBuffer(LedConstants.LENGTH);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    
    //general color methods : use variables
    public void setRGB(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values
            m_ledBuffer.setRGB(i, r, g, b);
         }
         
         m_led.setData(m_ledBuffer);
    }
    
    public void setHSV(int h, int s, int v) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values
            m_ledBuffer.setHSV(i, h, s, v);
         }
         
         m_led.setData(m_ledBuffer);
    }

    //custom color methods : use constants
    public void setColorYelllow() {
        setHSV(Yellow.HUE, Yellow.SATURATION, Yellow.VALUE);
    }

    public void setColorPurple() {
        setHSV(Purple.HUE, Purple.SATURATION, Purple.VALUE);
    }



    
}
