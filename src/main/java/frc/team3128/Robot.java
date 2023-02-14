// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3128.autonomous.AutoPrograms;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team3128.common.utility.NAR_Shuffleboard;
import java.util.Random;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {

    public static RobotContainer m_robotContainer = new RobotContainer();
    private Command m_autonomousCommand;
    public static AutoPrograms autoPrograms = new AutoPrograms();
    public static int value;

    @Override
    public void robotInit(){
        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotPeriodic(){
        m_robotContainer.updateDashboard();        
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.init();
        m_autonomousCommand = autoPrograms.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        m_robotContainer.init();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationInit() {
        // gyro = new WPI_Pigeon2(0, "drivetrain");
        // gyro.configFactoryDefault();
        
        // Starts recording to data log
        DataLogManager.start();

        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());

        // (alternatively) Record only DS control data
        DriverStation.startDataLog(DataLogManager.getLog(), false);
        // NAR_Shuffleboard.addData("Drivetrain","Pose",() -> ("0"),2,0,4,1);
        // NAR_Shuffleboard.addComplex("Drivetrain","Gyro",gyro,3,1,2,2);//.withWidget("Gyro");
    }

    @Override
    public void simulationPeriodic() {
        CommandScheduler.getInstance().run();
        value = (new Random()).nextInt(20);

    }
    
    @Override
    public void disabledPeriodic() {

    }
}
