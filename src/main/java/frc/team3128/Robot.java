// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import org.littletonrobotics.junction.LoggedRobot;

// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {
    public static Robot instance;

    public static RobotContainer m_robotContainer = new RobotContainer();
    private Command m_autonomousCommand;
    public static AutoPrograms autoPrograms = new AutoPrograms();
    public Timer timer;
    public Timer xlockTimer;
    public double startTime;

    public static synchronized Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    @Override
    public void robotInit(){
        LiveWindow.disableAllTelemetry();
        // Pivot.getInstance().offset = Pivot.getInstance().getAngle();
    }

    @Override
    public void robotPeriodic(){
        m_robotContainer.updateDashboard();        
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.init();
        timer = new Timer();
        m_autonomousCommand = autoPrograms.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
            timer.start();
        }
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
        if (timer.hasElapsed(14.75)) {
            new RunCommand(()->Swerve.getInstance().xlock(), Swerve.getInstance()).schedule();
        }
    }

    @Override
    public void teleopInit() {
        m_robotContainer.init();
        xlockTimer = new Timer();
        xlockTimer.start();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        if (xlockTimer.hasElapsed(134.75)) {
            //new RunCommand(()->Swerve.getInstance().xlock(), Swerve.getInstance()).schedule();
        }
    }

    @Override
    public void simulationInit() {
        
    }

    @Override
    public void simulationPeriodic() {
        CommandScheduler.getInstance().run();
    }
    
    @Override
    public void disabledPeriodic() {
        // Telescope.getInstance().engageBrake();
    }
}
