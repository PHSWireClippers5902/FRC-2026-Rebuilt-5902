package org.frc5902.robot.subsystems.SLAMtake.slam;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import org.frc5902.robot.subsystems.SLAMtake.slam.SlamSystemConstants.SlamConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.frc5902.robot.util.motorutil.SparkUtil.tryUntilOk;

public class SlamIOSpark implements SlamIO {
    // hardware
    public final SparkBase Slam;
    public final RelativeEncoder SlamEncoder;
    public final SparkClosedLoopController SlamController;
    public final DigitalInput limitSwitch;
    // signals
    public final DoubleSupplier position;
    public final DoubleSupplier velocity;
    public final DoubleSupplier appliedVolts;
    public final DoubleSupplier temp;
    public final DoubleSupplier current;
    public final BooleanSupplier LimitSwitchPressed;
    // outputs
    public final Debouncer SlamConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    public final Debouncer LimitSwitchDebounce = new Debouncer(0.05, DebounceType.kFalling);

    public SlamIOSpark() {
        Slam = new SparkMax(SlamConstants.SlamCANID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(SlamConstants.SlamPositionConversionFactor);
        config.encoder.velocityConversionFactor(SlamConstants.SlamVelocityConversionFactor);
        config.closedLoop.positionWrappingEnabled(false);
        config.inverted(SlamConstants.inverted)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(SlamConstants.StallLimit, SlamConstants.FreeLimit);
        config.closedLoop.pid(
                SlamConstants.SlamPID.getProportional(),
                SlamConstants.SlamPID.getIntegral(),
                SlamConstants.SlamPID.getDeriviative());

        tryUntilOk(
                Slam,
                5,
                () -> Slam.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        SlamEncoder = Slam.getEncoder();
        tryUntilOk(Slam, 5, () -> SlamEncoder.setPosition(0.0));
        SlamController = Slam.getClosedLoopController();

        position = () -> SlamEncoder.getPosition();
        velocity = () -> SlamEncoder.getVelocity();
        appliedVolts = () -> Slam.getAppliedOutput();
        temp = () -> Slam.getMotorTemperature();
        current = () -> Slam.getOutputCurrent();

        limitSwitch = new DigitalInput(SlamConstants.SlamLimitSwitchID);
        LimitSwitchPressed = () -> (!limitSwitch.get());

        resetEncoderToPosition(SlamConstants.EstimatedTopValue);
    }

    @Override
    public void updateInputs(SlamIOInputs inputs) {
        inputs.data = new SlamIOData(
                SlamConnectedDebounce.calculate(Slam.getLastError() == REVLibError.kOk),
                LimitSwitchDebounce.calculate(LimitSwitchPressed.getAsBoolean()),
                position.getAsDouble(),
                velocity.getAsDouble(),
                appliedVolts.getAsDouble(),
                temp.getAsDouble(),
                current.getAsDouble());
    }

    @Override
    public void runVolts(double volts) {
        Slam.setVoltage(volts);
    }

    @Override
    public void runRotationsPerSecond(double rotationsPerSecond) {
        SlamController.setSetpoint(rotationsPerSecond, ControlType.kVelocity);
    }

    @Override
    public void runAngle(double rotations) {
        SlamController.setSetpoint(rotations, ControlType.kPosition);
    }

    @Override
    public void resetEncoderToPosition(double position) {
        SlamEncoder.setPosition(position);
    }

    @Override
    public void stop() {
        Slam.stopMotor();
    }
}
