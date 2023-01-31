package frc.team3128;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdDriveUp;
import frc.team3128.commands.CmdGyroBalance;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.commands.CmdTargetPursuit;
import frc.team3128.common.hardware.camera.Camera;
import frc.team3128.common.hardware.camera.NAR_Camera;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    private Vision vision;
    private NAR_Camera cam;
    private Pivot pivot;

    private NAR_Joystick leftStick;
    private NAR_Joystick rightStick;

    private NAR_XboxController controller;

    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  
    private boolean DEBUG = true; 

    private Trigger hasTarget;

    public RobotContainer() {
        vision = Vision.getInstance();
        // ConstantsInt.initTempConstants();
        swerve = Swerve.getInstance();
        pivot = Pivot.getInstance();


        //TODO: Enable all PIDSubsystems so that useOutput runs here

        leftStick = new NAR_Joystick(0);
        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);

        //commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(rightStick::getX, rightStick::getY, rightStick::getZ, rightStick::getThrottle, true));
        commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, rightStick::getThrottle, true));
        initDashboard();
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        rightStick.getButton(1).onTrue(new InstantCommand(()->swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))));
        rightStick.getButton(2).onTrue(new InstantCommand(swerve::toggle));

        rightStick.getButton(3).onTrue(new SequentialCommandGroup(
            new CmdDriveUp(),
            new CmdGyroBalance()
        ));
        // rightStick.getButton(3).onTrue(new InstantCommand(()->swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))));
        // rightStick.getButton(4).onTrue(new CmdAlign()).onFalse(new InstantCommand(()-> swerve.stop()));
        // rightStick.getButton(5).onTrue(new InstantCommand(()->swerve.resetOdometry(vision.robotPos(Camera.SHOOTER.hostname))));
        // rightStick.getButton(6).onTrue(new CmdTargetPursuit(Camera.SHOOTER.hostname)).onFalse(new InstantCommand(()->swerve.stop(),swerve));

        // rightStick.getButton(6).whenActive(new InstantCommand(()-> {
        //     if(vision.hasValidTarget(Camera.SHOOTER.hostname)) {
        //         Trajectories.lineCmd(swerve.getPose(),vision.targetPos(Camera.SHOOTER.hostname, swerve.getPose()));
        //     }
        // })).whenInactive(new InstantCommand(swerve::stop,swerve));

        // hasTarget = new Trigger(()-> vision.hasValidTarget(Camera.SHOOTER.hostname))
        // .whenActive(new RunCommand(()-> controller.setRumble(RumbleType.kLeftRumble,1)))
        // .whenInactive(new InstantCommand(()-> controller.setRumble(RumbleType.kLeftRumble, 0)));

        rightStick.getButton(7).onTrue(new InstantCommand(()->pivot.zeroEncoder()));

    }

    public void init() {

    }

    private void initDashboard() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            //SmartDashboard.putData("Swerve", swerve);
        }

        swerve.initShuffleboard();
        vision.initShuffleboard();

        NarwhalDashboard.startServer();   
        
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        for (NAR_Camera ll : vision.getCameras()) {
            NarwhalDashboard.addLimelight(ll);
            ll.setLED(false);
        }
    }

    public void updateDashboard() {
        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("x", swerve.getPose().getX());
        NarwhalDashboard.put("y", swerve.getPose().getY());
        SmartDashboard.putNumber("LeftX",controller.getLeftX());
        SmartDashboard.putNumber("LeftY",controller.getLeftY());
        SmartDashboard.putNumber("RightX",controller.getRightX());
        SmartDashboard.putNumber("RightY",controller.getRightY());
        NAR_Shuffleboard.update();
        SmartDashboard.putNumber("Pitch",swerve.getPitch());
    }
}
