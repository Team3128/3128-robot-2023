package frc.team3128;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.commands.CmdRetractIntake;
import frc.team3128.commands.CmdScore;
import frc.team3128.commands.CmdScoreOptimized;
import frc.team3128.commands.CmdShelfPickup;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMoveArm;
import frc.team3128.commands.CmdPickupOptimized;
import frc.team3128.commands.CmdBangBangBalance;
import frc.team3128.commands.CmdDriveUp;
import frc.team3128.commands.CmdExtendIntake;
import frc.team3128.commands.CmdGyroBalance;
import frc.team3128.commands.CmdManipGrab;
import frc.team3128.common.hardware.camera.NAR_Camera;
import frc.team3128.common.hardware.input.NAR_ButtonBoard;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Led;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import static frc.team3128.Constants.ArmConstants.*;

import java.util.function.BooleanSupplier;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    private Vision vision;
    private NAR_Camera cam;
    private Intake intake;
    private Pivot pivot;
    private Telescope telescope;
    private Manipulator manipulator;
    private Led led;

    private NAR_Joystick leftStick;
    private NAR_Joystick rightStick;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;

    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  
    public static BooleanSupplier DEBUG = ()-> false; 

    private Trigger hasTarget;

    public RobotContainer() {
        NAR_Shuffleboard.addData("DEBUG", "DEBUG", ()-> DEBUG.getAsBoolean(), 0, 1);
        var x = NAR_Shuffleboard.addData("DEBUG", "TOGGLE", false, 0, 0).withWidget("Toggle Button");
        DEBUG = ()-> x.getEntry().getBoolean(false);

        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        vision = Vision.getInstance();
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();
        led = Led.getInstance();

        //TODO: Enable all PIDSubsystems so that useOutput runs here
        // pivot.enable();
        // telescope.enable();

        leftStick = new NAR_Joystick(0);
        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);
        CmdMove.setController(controller::getLeftX, controller::getLeftY, controller::getRightX, ()-> Swerve.throttle);

        //commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(rightStick::getX, rightStick::getY, rightStick::getZ, true));
        commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true));
        initDashboard();
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        controller.getButton("A").onTrue(new InstantCommand(()-> Vision.AUTO_ENABLED = !Vision.AUTO_ENABLED));
        controller.getButton("RightTrigger").onTrue(new InstantCommand(()-> Swerve.throttle = 1)).onFalse(new InstantCommand(()-> Swerve.throttle = 0.8));
        controller.getButton("LeftTrigger").onTrue(new InstantCommand(()-> Swerve.throttle = .25)).onFalse(new InstantCommand(()-> Swerve.throttle = 0.8));
        controller.getButton("X").onTrue(new RunCommand(()-> swerve.xlock(), swerve)).onFalse(new InstantCommand(()-> swerve.stop(),swerve));
        
        rightStick.getButton(1).onTrue(new InstantCommand(()->swerve.resetOdometry(new Pose2d())));
        rightStick.getButton(2).onTrue(new InstantCommand(()->telescope.engageBrake()));
        rightStick.getButton(3).onTrue(new InstantCommand(()-> telescope.releaseBrake()));
        
        // zeroing
        rightStick.getButton(4).onTrue(new InstantCommand(()->telescope.zeroEncoder()));

        // shuffleboard things
        rightStick.getButton(5).onTrue(new InstantCommand(()->pivot.startPID(0)));
        rightStick.getButton(6).onTrue(new InstantCommand(()->telescope.startPID(11.5)));
        rightStick.getButton(7).onTrue(new SequentialCommandGroup(new CmdDriveUp(), new CmdBangBangBalance()));
        //rightStick.getButton(8).onTrue(new CmdMoveArm(ArmPosition.NEUTRAL, false));
        rightStick.getButton(8).onTrue(new CmdGyroBalance());
    
        // manual controls
        rightStick.getButton(9).onTrue(new InstantCommand(()->telescope.extend())).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        rightStick.getButton(10).onTrue(new InstantCommand(()->telescope.retract())).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        
        rightStick.getButton(11).onTrue(new InstantCommand(()->pivot.setPower(0.2))).onFalse(new InstantCommand(()->pivot.setPower(0.0)));
        rightStick.getButton(12).onTrue(new InstantCommand(()->pivot.setPower(-0.2))).onFalse(new InstantCommand(()->pivot.setPower(0.0)));

        rightStick.getButton(13).onTrue(new CmdManipGrab(true));
        rightStick.getButton(14).onTrue(new CmdManipGrab(false));
        rightStick.getButton(15).onTrue(new InstantCommand(() -> manipulator.stopRoller(), manipulator));
        rightStick.getButton(16).onTrue(new InstantCommand(() -> manipulator.openClaw(), manipulator));

        leftStick.getButton(1).onTrue(new CmdMoveArm(ArmPosition.NEUTRAL, false));
        leftStick.getButton(2).onTrue(new CmdPickupOptimized(true));
        leftStick.getButton(3).onTrue(new CmdPickupOptimized(false));

        //Intake Buttons
        leftStick.getButton(8).onTrue(new CmdExtendIntake()).onFalse(new CmdRetractIntake());
        leftStick.getButton(9).onTrue(new InstantCommand(()-> intake.enableRollersForward())).onFalse(new InstantCommand(()-> intake.disableRollers()));
        leftStick.getButton(10).onTrue(new InstantCommand(()-> intake.enableRollersReverse())).onFalse(new InstantCommand(()-> intake.disableRollers()));
        leftStick.getButton(11).onTrue(new InstantCommand(()-> intake.startPID(30)));
        leftStick.getButton(12).onTrue(new InstantCommand(()->intake.setIntake(0.2))).onFalse(new InstantCommand(()->intake.setIntake(0.0)));
        leftStick.getButton(13).onTrue(new InstantCommand(()->intake.setIntake(-0.2))).onFalse(new InstantCommand(()->intake.setIntake(0.0)));
        
        buttonPad.getButton(5).onTrue(new CmdScoreOptimized(ArmPosition.LOW_FLOOR, 1));
        buttonPad.getButton(8).onTrue(new CmdScoreOptimized(ArmPosition.MID_CUBE, 1));
        buttonPad.getButton(11).onTrue(new CmdScoreOptimized(ArmPosition.TOP_CUBE, 1));
        buttonPad.getButton(1).onTrue(new InstantCommand(()-> {
            Vision.SELECTED_GRID = DriverStation.getAlliance() == Alliance.Red ? 0 : 2;
        }));
        buttonPad.getButton(2).onTrue(new InstantCommand(()-> Vision.SELECTED_GRID = 1));
        buttonPad.getButton(3).onTrue(new InstantCommand(()-> {
            Vision.SELECTED_GRID = DriverStation.getAlliance() == Alliance.Red ? 2 : 0;
        }));

    }

    public void init() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            buttonPad.getButton(4).onTrue(new CmdScoreOptimized(ArmPosition.LOW_FLOOR, 0));
            buttonPad.getButton(6).onTrue(new CmdScoreOptimized(ArmPosition.LOW_FLOOR, 2));
            buttonPad.getButton(7).onTrue(new CmdScoreOptimized(ArmPosition.MID_CONE, 0));
            buttonPad.getButton(9).onTrue(new CmdScoreOptimized(ArmPosition.MID_CONE, 2));
            buttonPad.getButton(10).onTrue(new CmdScoreOptimized(ArmPosition.TOP_CONE, 0));
            buttonPad.getButton(12).onTrue(new CmdScoreOptimized(ArmPosition.TOP_CONE, 2));
            
        }
        else {
            buttonPad.getButton(6).onTrue(new CmdScoreOptimized(ArmPosition.LOW_FLOOR, 0));
            buttonPad.getButton(4).onTrue(new CmdScoreOptimized(ArmPosition.LOW_FLOOR, 2));
            buttonPad.getButton(9).onTrue(new CmdScoreOptimized(ArmPosition.MID_CONE, 0));
            buttonPad.getButton(7).onTrue(new CmdScoreOptimized(ArmPosition.MID_CONE, 2));
            buttonPad.getButton(12).onTrue(new CmdScoreOptimized(ArmPosition.TOP_CONE, 0));
            buttonPad.getButton(10).onTrue(new CmdScoreOptimized(ArmPosition.TOP_CONE, 2));
        }
    }

    private void initDashboard() {
        if (DEBUG.getAsBoolean()) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            //SmartDashboard.putData("Swerve", swerve);
        }

        swerve.initShuffleboard();
        vision.initShuffleboard();
        intake.initShuffleboard();
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
        SmartDashboard.putNumber("Pitch",swerve.getPitch());
    }
}
