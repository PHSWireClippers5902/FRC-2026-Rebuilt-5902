package org.frc5902.robot.subsystems.compbot.launcher.flywheel;

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
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5902.robot.subsystems.compbot.launcher.LauncherConstants.FlywheelConstants;

import java.util.function.DoubleSupplier;

import static org.frc5902.robot.util.SparkUtil.tryUntilOk;

public class FlywheelSpark implements FlywheelIO {
    // hardware
    public final SparkBase flywheel;
    public final RelativeEncoder flywheelencoder;
    public final SparkClosedLoopController flywheelcontroller;
    // signals
    public final DoubleSupplier position;
    public final DoubleSupplier velocity;
    public final DoubleSupplier appliedVolts;
    public final DoubleSupplier temp;

    // outputs
    public final Debouncer flywheelConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public FlywheelSpark() {
        flywheel = new SparkMax(FlywheelConstants.FlywheelCANID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(FlywheelConstants.flywheelPositionConversionFactor);
        config.closedLoop.positionWrappingEnabled(false);
        config.inverted(FlywheelConstants.inverted)
                .idleMode(FlywheelConstants.idleMode)
                .smartCurrentLimit(FlywheelConstants.StallLimit, FlywheelConstants.FreeLimit);
        config.closedLoop.pid(
                FlywheelConstants.flywheelPID.getProportional(),
                FlywheelConstants.flywheelPID.getIntegral(),
                FlywheelConstants.flywheelPID.getDeriviative());
        tryUntilOk(
                flywheel,
                5,
                () -> flywheel.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        flywheelencoder = flywheel.getEncoder();
        tryUntilOk(flywheel, 5, () -> flywheelencoder.setPosition(0.0));
        flywheelcontroller = flywheel.getClosedLoopController();

        position = () -> flywheelencoder.getPosition();
        velocity = () -> flywheelencoder.getVelocity();
        appliedVolts = () -> flywheel.getAppliedOutput();
        temp = () -> flywheel.getMotorTemperature();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.data = new FlywheelIOData(
                flywheelConnectedDebounce.calculate(flywheel.getLastError() != REVLibError.kOk),
                Rotation2d.fromRotations(position.getAsDouble()).getRadians(),
                Rotation2d.fromRotations(velocity.getAsDouble()).getRadians(),
                appliedVolts.getAsDouble(),
                temp.getAsDouble());
    }

    @Override
    public void runVolts(double volts) {
        flywheel.setVoltage(volts);
    }

    @Override
    public void runRadiansPerSecond(double radiansPerSecond) {
        flywheelcontroller.setSetpoint(radiansPerSecond, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        flywheel.stopMotor();
    }
}
