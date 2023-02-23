package frc.team3128;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.team3128.commands.CmdShelfPickup;
// import frc.team3128.commands.CmdShelfPickup;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMoveArm;
import frc.team3128.commands.CmdMoveScore;
import frc.team3128.commands.CmdExtendIntake;
import frc.team3128.commands.CmdGyroBalance;
import frc.team3128.commands.CmdHandoff;
import frc.team3128.commands.CmdManipulatorIntake;
import frc.team3128.commands.CmdManipulatorOutake;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.common.hardware.camera.NAR_Camera;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import frc.team3128.subsystems.Intake.IntakeState;
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

    private NAR_Joystick leftStick;
    private NAR_Joystick rightStick;
    private NAR_Joystick buttonPad;

    private NAR_XboxController controller;

    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  
    public static BooleanSupplier DEBUG = ()-> false; 

    private Trigger hasTarget;

    public RobotContainer() {
        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        vision = Vision.getInstance();
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();

        //TODO: Enable all PIDSubsystems so that useOutput runs here
        // pivot.enable();
        // telescope.enable();

        leftStick = new NAR_Joystick(0);
        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_Joystick(3);
        CmdMove.setController(controller::getLeftX, controller::getLeftY, controller::getRightX, ()-> Swerve.throttle);

        //commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(rightStick::getX, rightStick::getY, rightStick::getZ, rightStick::getThrottle, true));
        commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, rightStick::getThrottle, true));
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
        rightStick.getButton(4).onTrue(new InstantCommand(()->pivot.zeroEncoder()));

        // shuffleboard things
        rightStick.getButton(5).onTrue(new InstantCommand(()->pivot.startPID(0)));
        rightStick.getButton(6).onTrue(new InstantCommand(()->telescope.startPID(11.5)));
        rightStick.getButton(7).onTrue(new InstantCommand(()-> telescope.zeroEncoder()));
        rightStick.getButton(8).onTrue(new CmdMoveArm(ArmPosition.NEUTRAL));

        // manual controls
        rightStick.getButton(9).onTrue(new InstantCommand(()->telescope.extend())).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        rightStick.getButton(10).onTrue(new InstantCommand(()->telescope.retract())).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        
        rightStick.getButton(11).onTrue(new InstantCommand(()->pivot.setPower(0.2))).onFalse(new InstantCommand(()->pivot.setPower(0.0)));
        rightStick.getButton(12).onTrue(new InstantCommand(()->pivot.setPower(-0.2))).onFalse(new InstantCommand(()->pivot.setPower(0.0)));
        
        rightStick.getButton(13).onTrue(new CmdManipulatorIntake());
        rightStick.getButton(14).onTrue(new CmdManipulatorOutake());
        rightStick.getButton(15).onTrue(new CmdShelfPickup(VisionConstants.LOADING_ZONE[0]));
        rightStick.getButton(16).onTrue(new CmdShelfPickup(VisionConstants.LOADING_ZONE[1]));

        leftStick.getButton(1).onTrue(new CmdMoveArm(ArmPosition.NEUTRAL));
        leftStick.getButton(2).onTrue(new CmdShelfPickup());
        leftStick.getButton(3).onTrue(new InstantCommand(() -> manipulator.closeClaw()));
        leftStick.getButton(4).onTrue(new CmdShelfPickup(VisionConstants.LOADING_ZONE[0]));
        leftStick.getButton(5).onTrue(new CmdShelfPickup(VisionConstants.LOADING_ZONE[1]));
        leftStick.getButton(6).onTrue(new CmdShelfPickup(VisionConstants.LOADING_ZONE[2]));

        //Intake Buttons
        leftStick.getButton(7).onTrue(new CmdHandoff());
        leftStick.getButton(8).onTrue(new CmdExtendIntake()).onFalse(new CmdRetractIntake());
        leftStick.getButton(9).onTrue(new InstantCommand(()-> intake.enableRollersForward())).onFalse(new InstantCommand(()-> intake.disableRollers()));
        leftStick.getButton(10).onTrue(new InstantCommand(()-> intake.enableRollersReverse())).onFalse(new InstantCommand(()-> intake.disableRollers()));
        leftStick.getButton(11).onTrue(new InstantCommand(()-> intake.startPID(30)));
        leftStick.getButton(12).onTrue(new InstantCommand(()-> intake.resetEncoders(0)));

        // for (int i = 0; i < VisionConstants.LOADING_ZONE.length; i++) {
        //     leftStick.getButton(i + 1).onTrue(new CmdMove(CmdMove.Type.LOADING, true, VisionConstants.LOADING_ZONE[i])).onFalse(new InstantCommand(()->swerve.stop(),swerve));
        // }
        // grid system
        
        buttonPad.getButton(5).onTrue(new CmdScore(false, ArmPosition.LOW_FLOOR, VisionConstants.RAMP_OVERRIDE[1], VisionConstants.SCORES_GRID[1]));
        buttonPad.getButton(8).onTrue(new CmdScore(false, ArmPosition.MID_CUBE, VisionConstants.RAMP_OVERRIDE[1], VisionConstants.SCORES_GRID[1]));
        buttonPad.getButton(11).onTrue(new CmdScore(false, ArmPosition.TOP_CUBE, VisionConstants.RAMP_OVERRIDE[1], VisionConstants.SCORES_GRID[1]));
        buttonPad.getButton(1).onTrue(new InstantCommand(()-> {
            Vision.SELECTED_GRID = DriverStation.getAlliance() == Alliance.Red ? 0 : 2;
        }));
        buttonPad.getButton(2).onTrue(new InstantCommand(()-> Vision.SELECTED_GRID = 1));
        buttonPad.getButton(3).onTrue(new InstantCommand(()-> {
            Vision.SELECTED_GRID = DriverStation.getAlliance() == Alliance.Red ? 2 : 0;
        }));

        // non-grid system
        // for (int i = 0; i < VisionConstants.SCORES.length; i++) {
        //     leftStick.getButton(i + 1).onTrue(new CmdMove(CmdMove.Type.SCORE, true, VisionConstants.SCORE_SETUP[i/3],VisionConstants.SCORES[i])).onFalse(new InstantCommand(()->swerve.stop(),swerve));
        // }
    }

    public void init() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            buttonPad.getButton(4).onTrue(new CmdScore(false, ArmPosition.LOW_FLOOR, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0]));
            buttonPad.getButton(6).onTrue(new CmdScore(false, ArmPosition.LOW_FLOOR, VisionConstants.RAMP_OVERRIDE[2], VisionConstants.SCORES_GRID[2]));
            buttonPad.getButton(7).onTrue(new CmdScore(false, ArmPosition.MID_CONE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0]));
            buttonPad.getButton(9).onTrue(new CmdScore(false, ArmPosition.MID_CONE, VisionConstants.RAMP_OVERRIDE[2], VisionConstants.SCORES_GRID[2]));
            buttonPad.getButton(10).onTrue(new CmdScore(false, ArmPosition.TOP_CONE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0]));
            buttonPad.getButton(12).onTrue(new CmdScore(false, ArmPosition.TOP_CONE, VisionConstants.RAMP_OVERRIDE[2], VisionConstants.SCORES_GRID[2]));
        }
        else {
            buttonPad.getButton(6).onTrue(new CmdScore(false, ArmPosition.LOW_FLOOR, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0]));
            buttonPad.getButton(4).onTrue(new CmdScore(false, ArmPosition.LOW_FLOOR, VisionConstants.RAMP_OVERRIDE[2], VisionConstants.SCORES_GRID[2]));
            buttonPad.getButton(9).onTrue(new CmdScore(false, ArmPosition.MID_CONE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0]));
            buttonPad.getButton(7).onTrue(new CmdScore(false, ArmPosition.MID_CONE, VisionConstants.RAMP_OVERRIDE[2], VisionConstants.SCORES_GRID[2]));
            buttonPad.getButton(12).onTrue(new CmdScore(false, ArmPosition.TOP_CONE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0]));
            buttonPad.getButton(10).onTrue(new CmdScore(false, ArmPosition.TOP_CONE, VisionConstants.RAMP_OVERRIDE[2], VisionConstants.SCORES_GRID[2]));
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
        NAR_Shuffleboard.addData("DEBUG", "DEBUG", ()-> DEBUG.getAsBoolean(), 0, 1);
        var x = NAR_Shuffleboard.addData("DEBUG", "TOGGLE", false, 0, 0).withWidget("Toggle Button");
        DEBUG = ()-> x.getEntry().getBoolean(false);
        
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
