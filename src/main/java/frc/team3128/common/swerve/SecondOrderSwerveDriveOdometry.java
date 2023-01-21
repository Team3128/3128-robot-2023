package frc.team3128.common.swerve;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;

public class SecondOrderSwerveDriveOdometry {
  private final SecondOrderSwerveDriveKinematics m_kinematics;
  private Pose2d m_poseMeters;
  private double m_prevTimeSeconds = -1;

  private Rotation2d m_gyroOffset;
  private Rotation2d m_previousAngle;

  private long m_previousTime = 0;
  private SwerveModuleState[] m_previousStates;

  /**
   * Constructs a SwerveDriveOdometry object.
   *
   * @param kinematics The swerve drive kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param initialPose The starting position of the robot on the field.
   */
  public SecondOrderSwerveDriveOdometry(
    SecondOrderSwerveDriveKinematics kinematics, Rotation2d gyroAngle, Pose2d initialPose) {
    m_kinematics = kinematics;
    m_poseMeters = initialPose;
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = initialPose.getRotation();
    MathSharedStore.reportUsage(MathUsageId.kOdometry_SwerveDrive, 1);
  }

  /**
   * Constructs a SwerveDriveOdometry object with the default pose at the origin.
   *
   * @param kinematics The swerve drive kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   */
  public SecondOrderSwerveDriveOdometry(SecondOrderSwerveDriveKinematics kinematics, Rotation2d gyroAngle) {
    this(kinematics, gyroAngle, new Pose2d());
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param pose The position on the field that your robot is at.
   * @param gyroAngle The angle reported by the gyroscope.
   */
  public void resetPosition(Pose2d pose, Rotation2d gyroAngle) {
    m_poseMeters = pose;
    m_previousAngle = pose.getRotation();
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose2d getPoseMeters() {
    return m_poseMeters;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose
   * over time. This method takes in the current time as a parameter to calculate period (difference
   * between two timestamps). The period is used to calculate the change in distance from a
   * velocity. This also takes in an angle parameter which is used instead of the angular rate that
   * is calculated from forward kinematics.
   *
   * @param currentTimeSeconds The current time in seconds.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param moduleStates The current state of all swerve modules. Please provide the states in the
   *     same order in which you instantiated your SwerveDriveKinematics.
   * @return The new pose of the robot.
   */
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SecondOrderSwerveModuleState... moduleStates) {
    double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
    m_prevTimeSeconds = currentTimeSeconds;

    var angle = gyroAngle.plus(m_gyroOffset);

    var chassisState = m_kinematics.toChassisSpeeds(moduleStates);
    var newPose =
        m_poseMeters.exp(
            new Twist2d(
                chassisState.vxMetersPerSecond * period + 0.5*chassisState.axMetersPerSecondSq * period * period,
                chassisState.vyMetersPerSecond * period + 0.5*chassisState.ayMetersPerSecondSq * period * period,
                angle.minus(m_previousAngle).getRadians()));

    m_previousAngle = angle;
    m_poseMeters = new Pose2d(newPose.getTranslation(), angle);

    return m_poseMeters;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose
   * over time. This method automatically calculates the current time to calculate period
   * (difference between two timestamps). The period is used to calculate the change in distance
   * from a velocity. This also takes in an angle parameter which is used instead of the angular
   * rate that is calculated from forward kinematics.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param moduleStates The current state of all swerve modules. Please provide the states in the
   *     same order in which you instantiated your SwerveDriveKinematics.
   * @return The new pose of the robot.
   */
  public Pose2d update(Rotation2d gyroAngle, SwerveModuleState... moduleStates) {

    if(m_previousStates.length != moduleStates.length) {
      throw new IllegalArgumentException("The number of module states must be the same as the number of previous states");
    }

    if(m_previousStates == null) {
      m_previousStates = moduleStates;
    }

    long dt = (WPIUtilJNI.now() - m_previousTime) / 1000000;

    SecondOrderSwerveModuleState[] secondOrderModuleStates = new SecondOrderSwerveModuleState[moduleStates.length];
    for(int i = 0; i < moduleStates.length; i++) {
        secondOrderModuleStates[i] = getSecondOrderSwerveModuleState(m_previousStates[i], moduleStates[i], dt)
    }

    m_previousTime = WPIUtilJNI.now();
    m_previousStates = moduleStates;

    return updateWithTime(WPIUtilJNI.now() * 1.0e-6, gyroAngle, secondOrderModuleStates);
  }

  public SecondOrderSwerveModuleState getSecondOrderSwerveModuleState(SwerveModuleState before, SwerveModuleState now, double dt) {
    double dv = now.speedMetersPerSecond - before.speedMetersPerSecond;
    Rotation2d dtheta = now.angle.minus(before.angle);
    return new SecondOrderSwerveModuleState(now.speedMetersPerSecond, now.angle, dtheta.times(1/dt), dv/dt);
  }
}