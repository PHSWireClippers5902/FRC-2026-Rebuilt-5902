// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5902.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5902.robot.Constants.RobotConstants;
import org.frc5902.robot.containers.CompRobotContainer;
import org.frc5902.robot.containers.RobotContainer;
import org.frc5902.robot.state.RobotState;
import org.frc5902.robot.util.buildutil.BuildInfo;
import org.frc5902.robot.util.buildutil.SystemTimeValidReader;
import org.frc5902.robot.util.buildutil.VirtualSubsystem;
import org.frc5902.robot.util.shifts.Phases;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

public class Robot extends LoggedRobot {
    private static final double loopOverrunWarningTimeout = 0.2;
    private static final double canErrorTimeThreshold = 0.5;
    private static final double lowBatteryVoltage = 11.8;
    private static final double lowBatteryDisabledTime = 1.5;
    private static final double lowBatteryMinCycleCount = 10;
    private static int lowBatteryCycleCount = 0;
    private Command autonomousCommand = null;
    private RobotContainer robotContainer;
    private double autoStart;
    private boolean autoMessagePrinted;
    private final Timer disabledTimer = new Timer();
    private final Timer canInitialErrorTimer = new Timer();
    private final Timer canErrorTimer = new Timer();

    private final Alert canErrorAlert =
            new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);

    private final Alert lowBatteryAlert = new Alert(
            "Battery voltage is very low, consider turning off the robot or replacing the battery.",
            AlertType.kWarning);
    private final Alert jitAlert = new Alert("Please wait to enable, JITing in progress.", AlertType.kWarning);

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
        AutoLogOutputManager.addObject(RobotState.getInstance());

        Logger.start();
        SignalLogger.enableAutoLogging(false);

        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(loopOverrunWarningTimeout);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

        SystemTimeValidReader.start();

        DriverStation.silenceJoystickConnectionWarning(true);

        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };

        CommandScheduler.getInstance()
                .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
        CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
        CommandScheduler.getInstance()
                .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

        canInitialErrorTimer.restart();
        canErrorTimer.restart();
        disabledTimer.restart();

        RobotController.setBrownoutVoltage(6.0);

        robotContainer = new CompRobotContainer();
    }

    @Override
    public void robotPeriodic() {
        VirtualSubsystem.runAllPeriodic();
        CommandScheduler.getInstance().run();
        VirtualSubsystem.runAllPeriodicAfterScheduler();

        if (autonomousCommand != null) {
            if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    System.out.printf("*** Auto finished in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
                } else {
                    System.out.printf("*** Auto cancelled in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
                }
                autoMessagePrinted = true;
            }
        }

        robotContainer.updateAlerts();
        robotContainer.updateDashboardOutputs();

        var canStatus = RobotController.getCANStatus();
        if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
            canErrorTimer.restart();
        }
        canErrorAlert.set(!canErrorTimer.hasElapsed(canErrorTimeThreshold)
                && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

        lowBatteryCycleCount += 1;
        if (DriverStation.isEnabled()) {
            disabledTimer.reset();
        }
        if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
                && disabledTimer.hasElapsed(lowBatteryDisabledTime)
                && lowBatteryCycleCount >= lowBatteryMinCycleCount) {
            lowBatteryAlert.set(true);
        }

        jitAlert.set(isJITing());

        RobotState.getInstance().periodic();
    }

    @Override
    public void robotInit() {
        Phases.getInstance();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autoStart = Timer.getTimestamp();
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        // notify phases that autonomous has ended
        Phases.getInstance().teleopInit();
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

    public static boolean isJITing() {
        return Timer.getTimestamp() < 45.0;
    }
}
