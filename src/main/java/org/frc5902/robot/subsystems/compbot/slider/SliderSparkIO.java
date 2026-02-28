package org.frc5902.robot.subsystems.compbot.slider;

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
import edu.wpi.first.wpilibj.DigitalInput;

import org.frc5902.robot.subsystems.compbot.slider.SliderConstants;
import org.frc5902.robot.util.buildutil.LoggedTunableNumber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.frc5902.robot.util.motorutil.SparkUtil.tryUntilOk;

public class SliderSparkIO implements SliderIO {
    // hardware
    public final SparkBase slider;
    public final RelativeEncoder sliderEncoder;
    public final SparkClosedLoopController sliderController;
    // signals
    public final DoubleSupplier position;
    public final DoubleSupplier velocity;
    public final DoubleSupplier appliedVolts;
    public final DoubleSupplier temp;

    public final DigitalInput sliderLimitSwitch;

    public final BooleanSupplier limitSwitchValue;
    public final Debouncer limitSwitchDebouncer = new Debouncer(0.1, DebounceType.kFalling);
    // outputs
    public final Debouncer sliderConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public SliderSparkIO() {
        slider = new SparkMax(SliderConstants.SliderCANID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(SliderConstants.SliderPositionConversionFactor);
        config.closedLoop.positionWrappingEnabled(false);
        config.inverted(SliderConstants.inverted)
                .idleMode(SliderConstants.idleMode)
                .smartCurrentLimit(SliderConstants.StallLimit, SliderConstants.FreeLimit);
        config.closedLoop.pid(
                SliderConstants.SliderPID.getProportional(),
                SliderConstants.SliderPID.getIntegral(),
                SliderConstants.SliderPID.getDeriviative());

        tryUntilOk(
                slider,
                5,
                () -> slider.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        sliderEncoder = slider.getEncoder();
        tryUntilOk(slider, 5, () -> sliderEncoder.setPosition(0.0));
        sliderController = slider.getClosedLoopController();

        sliderLimitSwitch = new DigitalInput(SliderConstants.LimitSwitchPort);

        position = () -> sliderEncoder.getPosition();
        velocity = () -> sliderEncoder.getVelocity();
        appliedVolts = () -> slider.getAppliedOutput();
        temp = () -> slider.getMotorTemperature();
        limitSwitchValue = () -> sliderLimitSwitch.get();
    }

    @Override
    public void updateInputs(SliderIOInputs inputs) {
        inputs.data = new SliderIOData(
                sliderConnectedDebounce.calculate(slider.getLastError() != REVLibError.kOk),
                Rotation2d.fromRotations(position.getAsDouble()).getRadians(),
                Rotation2d.fromRotations(velocity.getAsDouble()).getRadians(),
                appliedVolts.getAsDouble(),
                temp.getAsDouble(),
                sliderConnectedDebounce.calculate(limitSwitchValue.getAsBoolean()));
    }

    
    public void runToPosition(double position) {
        sliderController.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void runVolts(double volts) {
        slider.setVoltage(volts);
    }

    @Override
    public void runRadiansPerSecond(double radiansPerSecond) {
        sliderController.setSetpoint(radiansPerSecond, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        slider.stopMotor();
    }
}
