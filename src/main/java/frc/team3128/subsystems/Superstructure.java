package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.PivotConstants;
import frc.team3128.Constants.TelescopeConstants;
import static frc.team3128.Constants.PivotConstants.*;

public class Superstructure extends SubsystemBase {

    private final Pivot pivot;
    private final Telescope telescope;

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
        TOP_CONE(180 - 81.666, 56.75), 
        TOP_CUBE(180 - 92.221, 56.75), 
        MID_CONE(180 - 95.559, 40.027), 
        MID_CUBE(180 - 110.041, 39.031), 
        LOW_FLOOR(180 - 155.114, 16.0), 
        HP_PICK_UP(0.0, 16.0), 
        INT_PICK_UP(0.0, 16.0), 
        NEUTRAL(0.0, 16.0);

        public double pivotAngle;
        public double teleDist;

        private ScoringPosition(double pivotAngle, double teleDist) {
            this.pivotAngle = pivotAngle;
            this.teleDist = teleDist;
        }
    }

    // soft limits don't work
    // TODO: stop tele/pivot (ie set them to stay where they are at pid) when it is at max

    //stop telescope and/or pivot if they are at the max
    public void checkMax() {
        if (telescope.getMeasurement() >= TelescopeConstants.MAX_DIST) {
            // telescope.stopTele(); // set to max
            telescope.startPID(ScoringPosition.TOP_CONE.teleDist);
        }
        
        if (pivot.getMeasurement() >= PivotConstants.MAX_ANGLE) {
            // pivot.stopPivot();
            pivot.startPID(ScoringPosition.TOP_CONE.pivotAngle);
        }
    } 

    public void checkMin() {
        if (telescope.getMeasurement() <= TelescopeConstants.MIN_DIST) {
            telescope.startPID(ScoringPosition.NEUTRAL.teleDist);
        }
        
        if (pivot.getMeasurement() <= PivotConstants.MIN_ANGLE) {
            pivot.startPID(ScoringPosition.NEUTRAL.pivotAngle);
        }
    } 

    public void checkAngleLength() {
         if (telescope.getMeasurement() > (PIVOT_HEIGHT/Math.cos(pivot.getMeasurement()))) {
            telescope.startPID(PIVOT_HEIGHT/Math.cos(pivot.getMeasurement()));
         }
    }


    
}
