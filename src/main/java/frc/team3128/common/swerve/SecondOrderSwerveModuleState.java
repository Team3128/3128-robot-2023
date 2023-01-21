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

    public void optimize(Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), angle.getDegrees());
        double targetSpeed = speedMetersPerSecond;
        double acceleration = accelerationMetersPerSecondSquared;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            acceleration = -acceleration;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        speedMetersPerSecond = targetSpeed;
        angle = Rotation2d.fromDegrees(targetAngle);
        accelerationMetersPerSecondSquared = acceleration;
      }

      private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

}