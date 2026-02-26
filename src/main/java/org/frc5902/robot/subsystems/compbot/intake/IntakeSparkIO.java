package org.frc5902.robot.subsystems.compbot.intake;

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
import org.frc5902.robot.subsystems.compbot.intake.IntakeSystemConstants.IntakeConstants;

import java.util.function.DoubleSupplier;

import static org.frc5902.robot.util.motorutil.SparkUtil.tryUntilOk;

public class IntakeSparkIO implements IntakeIO {
    // hardware
    public final SparkBase Intake;
    public final RelativeEncoder IntakeEncoder;
    public final SparkClosedLoopController IntakeController;
    // signals
    public final DoubleSupplier position;
    public final DoubleSupplier velocity;
    public final DoubleSupplier appliedVolts;
    public final DoubleSupplier temp;

    // outputs
    public final Debouncer IntakeConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public IntakeSparkIO() {
        Intake = new SparkMax(IntakeConstants.IntakeCANID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(IntakeConstants.IntakePositionConversionFactor);
        config.closedLoop.positionWrappingEnabled(false);
        config.inverted(IntakeConstants.inverted)
                .idleMode(IntakeConstants.idleMode)
                .smartCurrentLimit(IntakeConstants.StallLimit, IntakeConstants.FreeLimit);
        config.closedLoop.pid(
                IntakeConstants.IntakePID.getProportional(),
                IntakeConstants.IntakePID.getIntegral(),
                IntakeConstants.IntakePID.getDeriviative());

        tryUntilOk(
                Intake,
                5,
                () -> Intake.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        IntakeEncoder = Intake.getEncoder();
        tryUntilOk(Intake, 5, () -> IntakeEncoder.setPosition(0.0));
        IntakeController = Intake.getClosedLoopController();

        position = () -> IntakeEncoder.getPosition();
        velocity = () -> IntakeEncoder.getVelocity();
        appliedVolts = () -> Intake.getAppliedOutput();
        temp = () -> Intake.getMotorTemperature();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.data = new IntakeIOData(
                IntakeConnectedDebounce.calculate(Intake.getLastError() != REVLibError.kOk),
                Rotation2d.fromRotations(position.getAsDouble()).getRadians(),
                Rotation2d.fromRotations(velocity.getAsDouble()).getRadians(),
                appliedVolts.getAsDouble(),
                temp.getAsDouble());
    }

    @Override
    public void runVolts(double volts) {
        Intake.setVoltage(volts);
    }

    @Override
    public void runRadiansPerSecond(double radiansPerSecond) {
        IntakeController.setSetpoint(radiansPerSecond, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        Intake.stopMotor();
    }
}
