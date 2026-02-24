package org.frc5902.robot.subsystems.compbot.agitator;

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

import java.util.function.DoubleSupplier;

import static org.frc5902.robot.util.SparkUtil.tryUntilOk;

public class AgitatorIOSpark implements AgitatorIO {
    // hardware
    public final SparkBase agitator;
    public final RelativeEncoder agitatorencoder;
    public final SparkClosedLoopController agitatorcontroller;
    // signals
    public final DoubleSupplier position;
    public final DoubleSupplier velocity;
    public final DoubleSupplier appliedVolts;
    public final DoubleSupplier temp;

    // outputs
    public final Debouncer agitatorConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public AgitatorIOSpark() {
        agitator = new SparkMax(AgitatorConstants.AgitatorCANID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(AgitatorConstants.agitatorPositionConversionFactor);
        config.closedLoop.positionWrappingEnabled(false);
        config.inverted(AgitatorConstants.inverted)
                .idleMode(AgitatorConstants.idleMode)
                .smartCurrentLimit(AgitatorConstants.StallLimit, AgitatorConstants.FreeLimit);
        config.closedLoop.pid(
                AgitatorConstants.agitatorPID.getProportional(),
                AgitatorConstants.agitatorPID.getIntegral(),
                AgitatorConstants.agitatorPID.getDeriviative());

        tryUntilOk(
                agitator,
                5,
                () -> agitator.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        agitatorencoder = agitator.getEncoder();
        tryUntilOk(agitator, 5, () -> agitatorencoder.setPosition(0.0));
        agitatorcontroller = agitator.getClosedLoopController();

        position = () -> agitatorencoder.getPosition();
        velocity = () -> agitatorencoder.getVelocity();
        appliedVolts = () -> agitator.getAppliedOutput();
        temp = () -> agitator.getMotorTemperature();
    }

    @Override
    public void updateInputs(AgitatorIOInputs inputs) {
        inputs.data = new AgitatorIOData(
                agitatorConnectedDebounce.calculate(agitator.getLastError() != REVLibError.kOk),
                Rotation2d.fromRotations(position.getAsDouble()).getRadians(),
                Rotation2d.fromRotations(velocity.getAsDouble()).getRadians(),
                appliedVolts.getAsDouble(),
                temp.getAsDouble());
    }

    @Override
    public void runVolts(double volts) {
        agitator.setVoltage(volts);
    }

    @Override
    public void runRadiansPerSecond(double radiansPerSecond) {
        agitatorcontroller.setSetpoint(radiansPerSecond, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        agitator.stopMotor();
    }
}
