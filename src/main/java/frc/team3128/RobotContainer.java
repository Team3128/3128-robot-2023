package frc.team3128;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdRetractArm;
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
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import frc.team3128.subsystems.Manipulator;


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
    private Telescope telescope;
    private Manipulator manipulator;

    private NAR_Joystick leftStick;
    private NAR_Joystick rightStick;

    private NAR_XboxController controller;

    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  
    private boolean DEBUG = true; 

    private Trigger hasTarget;

    public RobotContainer() {
        vision = Vision.getInstance();
        swerve = Swerve.getInstance();
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();

        //TODO: Enable all PIDSubsystems so that useOutput runs here
        pivot.enable();
        telescope.enable();

        leftStick = new NAR_Joystick(0);
        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);

        // commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(rightStick::getX, rightStick::getY, rightStick::getZ, rightStick::getThrottle, true));
        // commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, rightStick::getThrottle, true));

        initDashboard();
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        rightStick.getButton(1).onTrue(new InstantCommand(()->swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))));
        rightStick.getButton(2).onTrue(new InstantCommand(swerve::toggle));

        // zeroing
        rightStick.getButton(3).onTrue(new InstantCommand(()->telescope.zeroEncoder()));
        rightStick.getButton(4).onTrue(new InstantCommand(()->pivot.zeroEncoder()));

        // shuffleboard things
        // rightStick.getButton(5).onTrue(new InstantCommand(()->pivot.startPID(90)));
        // rightStick.getButton(6).onTrue(new InstantCommand(()->telescope.startPID(40)));

        // manual controls
        rightStick.getButton(9).whileTrue(new InstantCommand(()->telescope.extend())).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        rightStick.getButton(10).whileTrue(new InstantCommand(()->telescope.retract())).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        
        rightStick.getButton(11).whileTrue(new InstantCommand(()->pivot.setPower(0.3))).onFalse(new InstantCommand(()->pivot.setPower(0.0)));
        rightStick.getButton(12).whileTrue(new InstantCommand(()->pivot.setPower(-0.3))).onFalse(new InstantCommand(()->pivot.setPower(0.0)));
        
        rightStick.getButton(13).onTrue(new InstantCommand(() -> manipulator.openClaw()));
        rightStick.getButton(14).onTrue(new InstantCommand(() -> manipulator.closeClaw()));

        // command controls
        rightStick.getButton(15).whileTrue(new CmdRetractArm(40, 90));
        rightStick.getButton(16).whileTrue(new CmdRetractArm(18, 120));
        
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
        telescope.initShuffleboard();
        pivot.initShuffleboard();
        manipulator.initShuffleboard();

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
    }
}
