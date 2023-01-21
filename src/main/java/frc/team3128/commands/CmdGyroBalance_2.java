// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.team3128.commands;

import frc.team3128.Constants.SwerveConstants.*;
import frc.team3128.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class CmdGyroBalance_2 extends CommandBase {

    private final Swerve swerve;

    private double error;
    private double currentAngle;
    private double drivePower;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
    public CmdGyroBalance_2() {
        swerve = Swerve.getInstance();
        addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    this.currentAngle = swerve.getPitch();

    error = BEAM_BALANCED_GOAL_DEGREES - currentAngle;
    drivePower = -Math.min(BEAM_BALANACED_DRIVE_KP * error, 1);

    // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
    if (drivePower < 0) {
      drivePower *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }

    // Limit the max power
    if (Math.abs(drivePower) > 0.4) {
      drivePower = Math.copySign(0.4, drivePower);
    }

    swerve.drive(new Translation2d(drivePower,0), 0,true);
    
    // Debugging Print Statments
    System.out.println("Current Angle: " + currentAngle);
    System.out.println("Error " + error);
    System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}