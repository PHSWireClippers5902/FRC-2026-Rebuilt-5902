// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5902.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc5902.robot.Constants.RobotConstants;
import org.frc5902.robot.Constants.RobotInitializationConstants;
import org.frc5902.robot.containers.KitbotRobotContainer;
import org.frc5902.robot.containers.RobotContainer;
import org.frc5902.robot.util.BuildInfo;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand = null;

    private final RobotContainer m_robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildInfo.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildInfo.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildInfo.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildInfo.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildInfo.GIT_BRANCH);

        switch (BuildInfo.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (RobotConstants.currentMode) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                // physics sim
                Logger.addDataReceiver(new NT4Publisher());
            case REPLAY:
                // if you are replaying, set up source
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();

        m_robotContainer = new KitbotRobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void robotInit() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
