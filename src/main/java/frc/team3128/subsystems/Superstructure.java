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

    private static Superstructure instance = null;

    public static synchronized Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
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
                pivot.startPID(PivotAngles.TOP_CONE.angle);
                telescope.startPID(TeleDists.TOP_CONE.dist);
                break;
            case TOP_CUBE:
                pivot.startPID(PivotAngles.TOP_CUBE.angle);
                telescope.startPID(TeleDists.TOP_CUBE.dist);
                break;
            case MID_CONE:
                pivot.startPID(PivotAngles.MID_CONE.angle);
                telescope.startPID(TeleDists.MID_CONE.dist);
                break;
            case MID_CUBE:
                pivot.startPID(PivotAngles.MID_CUBE.angle);
                telescope.startPID(TeleDists.MID_CUBE.dist);
                break;
            case LOW_FLOOR: 
                pivot.startPID(PivotAngles.LOW_FLOOR.angle);
                telescope.startPID(TeleDists.LOW_FLOOR.dist);
                break;
            case HP_PICK_UP:
                pivot.startPID(PivotAngles.HP_PICK_UP.angle);
                telescope.startPID(TeleDists.HP_PICK_UP.dist);
                break;
            case INT_PICK_UP:
                pivot.startPID(PivotAngles.INT_PICK_UP.angle);
                telescope.startPID(TeleDists.INT_PICK_UP.dist);
                break;
            case NEUTRAL:
                pivot.startPID(PivotAngles.NEUTRAL.angle);
                telescope.startPID(TeleDists.NEUTRAL.dist);
                break;

        }
    }

    //stop telescope and/or pivot if they are at the max
    public void checkMax() {
        if (telescope.getMeasurement() >= TelescopeConstants.MAX_DIST) {
            // telescope.stopTele(); // set to max
            telescope.startPID(TeleDists.TOP_CONE.dist);
        }
        
        if (pivot.getMeasurement() >= PivotConstants.MAX_ANGLE) {
            // pivot.stopPivot();
            pivot.startPID(PivotAngles.TOP_CONE.angle);
        }
    } 

    public void checkMin() {
        if (telescope.getMeasurement() <= TelescopeConstants.MIN_DIST) {
            // telescope.stopTele(); // set to min
            telescope.startPID(TeleDists.NEUTRAL.dist);
        }
        
        if (pivot.getMeasurement() <= PivotConstants.MIN_ANGLE) {
            // pivot.stopPivot();
            pivot.startPID(PivotAngles.NEUTRAL.angle);
        }
    } 

    public void checkAngleLength() {
        // if (telescope.getMeasurement() > (Math.cos)))
    }
    
}
