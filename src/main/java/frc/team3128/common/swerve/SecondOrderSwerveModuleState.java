package frc.team3128.common.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SecondOrderSwerveModuleState extends SwerveModuleState {

    public Rotation2d angularVelocity;
    public double accelerationMetersPerSecondSquared;

    public SecondOrderSwerveModuleState() {}

    public SecondOrderSwerveModuleState(double speedMetersPerSecond, Rotation2d angle, Rotation2d angularVelocity, double accelerationMetersPerSecondSquared) {
        super(speedMetersPerSecond, angle);
        this.angularVelocity = angularVelocity;
        this.accelerationMetersPerSecondSquared = accelerationMetersPerSecondSquared;
    }

    @Override
    public boolean equals(Object obj) {
        return super.equals(obj) && angularVelocity.equals(angularVelocity) && accelerationMetersPerSecondSquared == accelerationMetersPerSecondSquared;
    }

    @Override
    public int hashCode() {
        return super.hashCode() + angularVelocity.hashCode() + Double.hashCode(accelerationMetersPerSecondSquared);
    }

    @Override
    public int compareTo(SwerveModuleState o) {
        return super.compareTo(o);
    }

    @Override
    public String toString() {
        return String.format(
            "SecondOrderSwerveModuleState(speedMetersPerSecond=%s, angle=%s, angularVelocity=%s, accelerationMetersPerSecondSquared=%s)",
            speedMetersPerSecond, angle, angularVelocity, accelerationMetersPerSecondSquared
        );
    }

}