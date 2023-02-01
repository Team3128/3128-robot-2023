package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.PivotConstants;
import frc.team3128.Constants.TelescopeConstants;
import frc.team3128.subsystems.Pivot.*;
import frc.team3128.subsystems.Telescope.*;
import frc.team3128.subsystems.Pivot.PivotAngles;
import static frc.team3128.Constants.PivotConstants.*;
import static frc.team3128.Constants.TelescopeConstants.*;

public class Superstructure extends SubsystemBase {

    private final Pivot pivot;
    private final Telescope telescope;

    private ScoringPosition m_ScoringPosition = ScoringPosition.NEUTRAL;


    public Superstructure() {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
    }

    private static Superstructure mInstance = null;

    public static synchronized Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    public enum ScoringPosition {
        TOP_CONE, 
        TOP_CUBE, 
        MID_CONE, 
        MID_CUBE, 
        LOW_FLOOR, 
        HP_PICK_UP, 
        INT_PICK_UP, 
        NEUTRAL
    }

    @Override
    public void periodic() {
        superstructureLoop();
    }

    public void superstructureLoop() {
        checkMin();
        checkMax();
        switch (m_ScoringPosition) {
            case TOP_CONE:
                pivot.startPID(PivotAngles.TOP_CONE);
                telescope.startPID(TeleDists.TOP_CONE);
                break;
            case TOP_CUBE:
                pivot.startPID(PivotAngles.TOP_CUBE);
                telescope.startPID(TeleDists.TOP_CUBE);
                break;
            case MID_CONE:
                pivot.startPID(PivotAngles.MID_CONE);
                telescope.startPID(TeleDists.MID_CONE);
                break;
            case MID_CUBE:
                pivot.startPID(PivotAngles.MID_CUBE);
                telescope.startPID(TeleDists.MID_CUBE);
                break;
            case LOW_FLOOR: 
                pivot.startPID(PivotAngles.LOW_FLOOR);
                telescope.startPID(TeleDists.LOW_FLOOR);
                break;
            case HP_PICK_UP:
                pivot.startPID(PivotAngles.HP_PICK_UP);
                telescope.startPID(TeleDists.HP_PICK_UP);
                break;
            case INT_PICK_UP:
                pivot.startPID(PivotAngles.INT_PICK_UP);
                telescope.startPID(TeleDists.INT_PICK_UP);
                break;
            case NEUTRAL:
                pivot.startPID(PivotAngles.NEUTRAL);
                telescope.startPID(TeleDists.NEUTRAL);
                break;

        }
    }



    //stop telescope and/or pivot if they are at the max
    public void checkMax() {
        if (telescope.getMeasurement() >= TelescopeConstants.MAX_DIST) {
            // telescope.stopTele(); // set to max
            telescope.startPID(TeleDists.TOP_CONE);
        }
        
        if (pivot.getMeasurement() >= PivotConstants.MAX_ANGLE) {
            // pivot.stopPivot();
            pivot.startPID(PivotAngles.TOP_CONE);
        }
    } 

    public void checkMin() {
        if (telescope.getMeasurement() <= TelescopeConstants.MIN_DIST) {
            // telescope.stopTele(); // set to min
            telescope.startPID(TeleDists.NEUTRAL);
        }
        
        if (pivot.getMeasurement() <= PivotConstants.MIN_ANGLE) {
            // pivot.stopPivot();
            pivot.startPID(PivotAngles.NEUTRAL);
        }
    } 

    public void checkAngleLength() {
        // if (telescope.getMeasurement() > (Math.cos)))
    }


    
}
