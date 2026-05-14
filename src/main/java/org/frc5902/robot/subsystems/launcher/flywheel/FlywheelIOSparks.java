package org.frc5902.robot.subsystems.launcher.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import org.frc5902.robot.subsystems.launcher.LauncherConstants;

import java.util.function.DoubleSupplier;

import static org.frc5902.robot.util.motorutil.SparkUtil.tryUntilOk;

public class FlywheelIOSparks implements FlywheelIO {
    // hardware
    public final SparkBase masterWheel;
    public final SparkBase followerWheel;
    public final RelativeEncoder flywheelencoder;
    public final SparkClosedLoopController flywheelcontroller;
    // signals
    public final DoubleSupplier position;
    public final DoubleSupplier velocity;
    public final DoubleSupplier appliedVolts;
    public final DoubleSupplier temp;
    public final DoubleSupplier current;
    // outputs
    public final Debouncer masterConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    public final Debouncer followerConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public FlywheelIOSparks() {

        masterWheel = new SparkMax(LauncherConstants.FlywheelLeftConstants.FlywheelCANID, MotorType.kBrushless);
        followerWheel = new SparkMax(LauncherConstants.FlywheelRightConstants.FlywheelCANID, MotorType.kBrushless);
        var masterConfig = new SparkMaxConfig();
        var followerConfig = new SparkMaxConfig();

        masterConfig.encoder.positionConversionFactor(
                LauncherConstants.FlywheelLeftConstants.flywheelPositionConversionFactor);
        masterConfig.encoder.velocityConversionFactor(
                LauncherConstants.FlywheelLeftConstants.flywheelVelocityConversionFactor);
        masterConfig.closedLoop.positionWrappingEnabled(false);
        masterConfig
                .inverted(LauncherConstants.FlywheelLeftConstants.inverted)
                .idleMode(LauncherConstants.FlywheelLeftConstants.idleMode)
                .smartCurrentLimit(
                        LauncherConstants.FlywheelLeftConstants.StallLimit,
                        LauncherConstants.FlywheelLeftConstants.FreeLimit);
        masterConfig.closedLoop.pid(
                LauncherConstants.FlywheelLeftConstants.flywheelPID.getProportional(),
                LauncherConstants.FlywheelLeftConstants.flywheelPID.getIntegral(),
                LauncherConstants.FlywheelLeftConstants.flywheelPID.getDeriviative());
        masterConfig.closedLoop.outputRange(0, 1);
        tryUntilOk(
                masterWheel,
                5,
                () -> masterWheel.configure(
                        masterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        flywheelencoder = masterWheel.getEncoder();
        tryUntilOk(masterWheel, 5, () -> flywheelencoder.setPosition(0.0));
        flywheelcontroller = masterWheel.getClosedLoopController();

        followerConfig.follow(LauncherConstants.FlywheelLeftConstants.FlywheelCANID, true);

        position = () -> flywheelencoder.getPosition();
        velocity = () -> flywheelencoder.getVelocity();
        appliedVolts = () -> masterWheel.getAppliedOutput();
        temp = () -> masterWheel.getMotorTemperature();
        current = () -> masterWheel.getOutputCurrent();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.data = new FlywheelIOData(
                masterConnectedDebounce.calculate(masterWheel.getLastError() == REVLibError.kOk),
                followerConnectedDebounce.calculate(followerWheel.getLastError() == REVLibError.kOk),
                position.getAsDouble(),
                velocity.getAsDouble(),
                appliedVolts.getAsDouble(),
                current.getAsDouble(),
                temp.getAsDouble());
    }

    @Override
    public void runVolts(double volts) {
        masterWheel.setVoltage(volts);
    }

    @Override
    public void runRotationsPerSecond(double rotationsPerSecond) {
        flywheelcontroller.setSetpoint(rotationsPerSecond, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        masterWheel.stopMotor();
    }
}
