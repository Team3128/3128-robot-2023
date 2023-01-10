package frc.team3128.subsystems;

import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.common.utility.NAR_Shuffleboard;

import static frc.team3128.Constants.HopperConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    enum GamePiece {
        CUBE,
        CONE_LEFT,
        CONE_RIGHT,
        NONE;
    }
    
    private NAR_TalonSRX m_serializer;

    private DigitalInput m_sensorLeft, m_sensorRight;

    private static Hopper instance;

    private Hopper() {
       configMotors();
       configSensors();
    }

    private void configMotors() {
        m_serializer = new NAR_TalonSRX(SERIALIZER_ID);
    }

    private void configSensors() {
        m_sensorLeft = new DigitalInput(SENSOR_LEFT_ID);
        m_sensorRight = new DigitalInput(SENSOR_RIGHT_ID);
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

    public boolean getSensorLeft() {
        return m_sensorLeft.get();
    }

    public boolean getSensorRight() {
        return m_sensorRight.get();
    }

    public GamePiece getGamePiece() {
        if (getSensorLeft() && getSensorRight()) return GamePiece.CUBE;
        if (getSensorLeft() && !getSensorRight()) return GamePiece.CONE_RIGHT;
        if (!getSensorLeft() && getSensorRight()) return GamePiece.CONE_LEFT;
        return GamePiece.NONE;
    }

    @Override
    public void periodic(){
        NAR_Shuffleboard.addData("Hopper", "Hopper Velocity", m_serializer.getSelectedSensorVelocity(), 1, 1);
        NAR_Shuffleboard.addData("Hopper", "Game Piece", getGamePiece(), 1, 2);
    }
}
