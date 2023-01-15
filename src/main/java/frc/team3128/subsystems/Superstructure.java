package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;


public class Superstructure extends SubsystemBase{

    private final Pivot pivot;
    private final Telescope telescope;

    public Superstructure() {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
    }

    private static Superstructure mInstance = null;

    public static Superstructure getInstance() {
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
        NEUTRAL;
    }

    @Override
    public void periodic() {
        
    }

    




    
}
