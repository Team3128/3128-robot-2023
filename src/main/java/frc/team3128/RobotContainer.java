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
import frc.team3128.commands.CmdRetractArm;
import frc.team3128.commands.CmdScore;
// import frc.team3128.commands.CmdShelfPickup;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMoveScore;
import frc.team3128.commands.CmdGyroBalance;
import frc.team3128.commands.CmdSwerveDrive;
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
import static frc.team3128.Constants.ArmConstants.*;

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
    private NAR_Joystick buttonPad;

    private NAR_XboxController controller;

    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  
    private boolean DEBUG = true; 

    private Trigger hasTarget;

    public RobotContainer() {
        swerve = Swerve.getInstance();
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
        CmdMove.setController(controller::getLeftX, controller::getLeftY, controller::getRightX, rightStick::getThrottle);

        //commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(rightStick::getX, rightStick::getY, rightStick::getZ, rightStick::getThrottle, true));
        //commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, rightStick::getThrottle, true));
        initDashboard();
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        rightStick.getButton(1).onTrue(new InstantCommand(()->swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))));
        rightStick.getButton(2).onTrue(new InstantCommand(()->telescope.engageBrake()));
        rightStick.getButton(3).onTrue(new InstantCommand(()-> telescope.releaseBrake()));
        

        // zeroing
       // rightStick.getButton(3).onTrue(new InstantCommand(()->telescope.zeroEncoder()));
        rightStick.getButton(4).onTrue(new InstantCommand(()->pivot.zeroEncoder()));

        // shuffleboard things
        rightStick.getButton(5).onTrue(new InstantCommand(()->pivot.startPID(0)));
        rightStick.getButton(6).onTrue(new InstantCommand(()->telescope.startPID(11.5)));
        rightStick.getButton(7).onTrue(new InstantCommand(()-> telescope.zeroEncoder()));


        // manual controls
        rightStick.getButton(9).onTrue(new InstantCommand(()->telescope.extend())).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        rightStick.getButton(10).onTrue(new InstantCommand(()->telescope.retract())).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        
        rightStick.getButton(11).onTrue(new InstantCommand(()->pivot.setPower(0.2))).onFalse(new InstantCommand(()->pivot.setPower(0.0)));
        rightStick.getButton(12).onTrue(new InstantCommand(()->pivot.setPower(-0.2))).onFalse(new InstantCommand(()->pivot.setPower(0.0)));
        
        rightStick.getButton(13).onTrue(new InstantCommand(() -> manipulator.openClaw()));
        rightStick.getButton(14).onTrue(new InstantCommand(() -> manipulator.closeClaw()));

        leftStick.getButton(1).onTrue(new CmdRetractArm());
        // leftStick.getButton(2).onTrue(new CmdShelfPickup());
        leftStick.getButton(3).onTrue(new InstantCommand(() -> manipulator.closeClaw()));

        leftStick.getButton(4).onTrue(new CmdScore(ScoringPosition.LOW_FLOOR, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0]));
        leftStick.getButton(5).onTrue(new CmdScore(ScoringPosition.LOW_FLOOR, VisionConstants.RAMP_OVERRIDE[1], VisionConstants.SCORES_GRID[1]));
        leftStick.getButton(6).onTrue(new CmdScore(ScoringPosition.LOW_FLOOR, VisionConstants.RAMP_OVERRIDE[2], VisionConstants.SCORES_GRID[2]));
        leftStick.getButton(7).onTrue(new CmdScore(ScoringPosition.MID_CONE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0]));
        leftStick.getButton(8).onTrue(new CmdScore(ScoringPosition.MID_CUBE, VisionConstants.RAMP_OVERRIDE[1], VisionConstants.SCORES_GRID[1]));
        leftStick.getButton(9).onTrue(new CmdScore(ScoringPosition.MID_CONE, VisionConstants.RAMP_OVERRIDE[2], VisionConstants.SCORES_GRID[2]));
        leftStick.getButton(10).onTrue(new CmdScore(ScoringPosition.TOP_CONE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0]));
        leftStick.getButton(11).onTrue(new CmdScore(ScoringPosition.TOP_CUBE, VisionConstants.RAMP_OVERRIDE[1], VisionConstants.SCORES_GRID[1]));
        leftStick.getButton(12).onTrue(new CmdScore(ScoringPosition.TOP_CONE, VisionConstants.RAMP_OVERRIDE[2], VisionConstants.SCORES_GRID[2]));
        // leftStick.getButton(1).onTrue(new InstantCommand(()-> telescope.startPID(20)));
        // leftStick.getButton(2).onTrue(new InstantCommand(()-> pivot.startPID(90)));

        // rightStick.getButton(1).onTrue(new InstantCommand(()->swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))));
        // rightStick.getButton(2).onTrue(new InstantCommand(swerve::toggle));
        // rightStick.getButton(4).onTrue(new InstantCommand(()->swerve.zeroGyro(180)));
        // for (int i = 0; i < VisionConstants.LOADING_ZONE.length; i++) {
        //     leftStick.getButton(i + 1).onTrue(new CmdMove(CmdMove.Type.LOADING, true, VisionConstants.LOADING_ZONE[i])).onFalse(new InstantCommand(()->swerve.stop(),swerve));
        // }
        // grid system
        
        buttonPad.getButton(4).onTrue(new CmdMoveScore(VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0])).onFalse(new InstantCommand(()->swerve.stop(),swerve));;
        buttonPad.getButton(5).onTrue(new CmdMoveScore(VisionConstants.RAMP_OVERRIDE[1], VisionConstants.SCORES_GRID[1])).onFalse(new InstantCommand(()->swerve.stop(),swerve));;
        buttonPad.getButton(6).onTrue(new CmdMoveScore(VisionConstants.RAMP_OVERRIDE[2], VisionConstants.SCORES_GRID[2])).onFalse(new InstantCommand(()->swerve.stop(),swerve));;
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
        SmartDashboard.putNumber("Pitch",swerve.getPitch());
    }
}
