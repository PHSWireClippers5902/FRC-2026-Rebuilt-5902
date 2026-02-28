package org.frc5902.robot.subsystems.compbot.launcher.inserter;

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
import org.frc5902.robot.subsystems.compbot.launcher.LauncherConstants.InserterConstants;

import java.util.function.DoubleSupplier;

import static org.frc5902.robot.util.motorutil.SparkUtil.tryUntilOk;

public class InserterIOSpark implements InserterIO {
    // hardware
    public final SparkBase inserter;
    public final RelativeEncoder inserterencoder;
    public final SparkClosedLoopController insertercontroller;
    // signals
    public final DoubleSupplier position;
    public final DoubleSupplier velocity;
    public final DoubleSupplier appliedVolts;
    public final DoubleSupplier temp;

    // outputs
    public final Debouncer inserterConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public InserterIOSpark() {
        inserter = new SparkMax(InserterConstants.InserterCANID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(InserterConstants.inserterPositionConversionFactor);
        config.closedLoop.positionWrappingEnabled(false);
        config.inverted(InserterConstants.inverted)
                .idleMode(InserterConstants.idleMode)
                .smartCurrentLimit(InserterConstants.StallLimit, InserterConstants.FreeLimit);
        config.closedLoop.pid(
                InserterConstants.inserterPID.getProportional(),
                InserterConstants.inserterPID.getIntegral(),
                InserterConstants.inserterPID.getDeriviative());

        tryUntilOk(
                inserter,
                5,
                () -> inserter.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        inserterencoder = inserter.getEncoder();
        tryUntilOk(inserter, 5, () -> inserterencoder.setPosition(0.0));
        insertercontroller = inserter.getClosedLoopController();

        position = () -> inserterencoder.getPosition();
        velocity = () -> inserterencoder.getVelocity();
        appliedVolts = () -> inserter.getAppliedOutput();
        temp = () -> inserter.getMotorTemperature();
    }

    @Override
    public void updateInputs(InserterIOInputs inputs) {
        inputs.data = new InserterIOData(
                inserterConnectedDebounce.calculate(inserter.getLastError() != REVLibError.kOk),
                Rotation2d.fromRotations(position.getAsDouble()).getRadians(),
                Rotation2d.fromRotations(velocity.getAsDouble()).getRadians(),
                appliedVolts.getAsDouble(),
                temp.getAsDouble());
    }

    @Override
    public void runVolts(double volts) {
        inserter.setVoltage(volts);
    }

    @Override
    public void runRadiansPerSecond(double radiansPerSecond) {
        insertercontroller.setSetpoint(radiansPerSecond, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        inserter.stopMotor();
    }
}
