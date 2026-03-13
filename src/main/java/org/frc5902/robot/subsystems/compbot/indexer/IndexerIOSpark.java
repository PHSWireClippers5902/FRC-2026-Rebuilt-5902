package org.frc5902.robot.subsystems.compbot.indexer;

import static org.frc5902.robot.util.motorutil.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import org.frc5902.robot.subsystems.compbot.indexer.IndexerConstants.IndexConstants;

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

public class IndexerIOSpark implements IndexerIO {
    // hardware
    public final SparkBase Indexer;
    public final RelativeEncoder IndexerEncoder;
    public final SparkClosedLoopController IndexerController;
    // signals
    public final DoubleSupplier position;
    public final DoubleSupplier velocity;
    public final DoubleSupplier appliedVolts;
    public final DoubleSupplier temp;
    public final DoubleSupplier current;
    // outputs
    public final Debouncer IndexerConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public IndexerIOSpark() {
        Indexer = new SparkMax(IndexConstants.IndexCANID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(IndexConstants.IndexPositionConversionFactor);
        config.encoder.velocityConversionFactor(IndexConstants.IndexVelocityConversionFactor);
        config.closedLoop.positionWrappingEnabled(false);
        config.inverted(IndexConstants.inverted)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(IndexConstants.StallLimit, IndexConstants.FreeLimit);
        config.closedLoop.pid(
                IndexConstants.IndexerPID.getProportional(),
                IndexConstants.IndexerPID.getIntegral(),
                IndexConstants.IndexerPID.getDeriviative());

        tryUntilOk(
                Indexer,
                5,
                () -> Indexer.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        IndexerEncoder = Indexer.getEncoder();
        tryUntilOk(Indexer, 5, () -> IndexerEncoder.setPosition(0.0));
        IndexerController = Indexer.getClosedLoopController();

        position = () -> IndexerEncoder.getPosition();
        velocity = () -> IndexerEncoder.getVelocity();
        appliedVolts = () -> Indexer.getAppliedOutput();
        temp = () -> Indexer.getMotorTemperature();
        current = () -> Indexer.getOutputCurrent();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.data = new IndexerIOData(
                IndexerConnectedDebounce.calculate(Indexer.getLastError() == REVLibError.kOk),
                position.getAsDouble(),
                velocity.getAsDouble(),
                appliedVolts.getAsDouble(),
                temp.getAsDouble(),
                current.getAsDouble());
    }

    @Override
    public void runVolts(double volts) {
        Indexer.setVoltage(volts);
    }

    @Override
    public void runRadiansPerSecond(double radiansPerSecond) {
        IndexerController.setSetpoint(radiansPerSecond, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        Indexer.stopMotor();
    }   
}
