package frc.team3128.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;

public class CmdCurrentCheck extends CommandBase {
    private NAR_TalonSRX motor;
    private double prevCurrent;
    private double threshold;
    private double absoluteThreshold;
    private double count;
    private boolean hasObject;

    public CmdCurrentCheck(NAR_TalonSRX motor, double threshold, double absoluteThreshold) {
        this.motor = motor;
        this.threshold = threshold;
        this.absoluteThreshold = absoluteThreshold;
    }

    @Override
    public void initialize() {
        prevCurrent = motor.getStatorCurrent();
        hasObject = false;
        count = 0;
    }

    @Override
    public void execute() {
        count++;
        if (count >= 5) {
            double current = motor.getStatorCurrent();
            SmartDashboard.putNumber("Current ROC", Math.abs(current - prevCurrent) / 0.1);
            if (Math.abs(current - prevCurrent) / 0.1 > threshold) {
                hasObject = true;
            }
            prevCurrent = current;
            if (Math.abs(current) > absoluteThreshold) hasObject = true;
            count = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return hasObject;
    }
}
