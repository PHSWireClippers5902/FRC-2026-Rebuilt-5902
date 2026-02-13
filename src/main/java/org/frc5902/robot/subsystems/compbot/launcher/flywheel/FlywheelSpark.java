package org.frc5902.robot.subsystems.compbot.launcher.flywheel;

import static org.frc5902.robot.util.SparkUtil.ifOk;
import static org.frc5902.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frc5902.robot.subsystems.compbot.launcher.LauncherConstants;
import org.frc5902.robot.subsystems.compbot.launcher.LauncherConstants.FlywheelConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FlywheelSpark implements FlywheelIO {
    // hardware
    public final SparkBase flywheel;
    public final RelativeEncoder flywheelencoder;
    // signals
    public final DoubleSupplier position;
    public final DoubleSupplier velocity;
    public final DoubleSupplier appliedVolts;
    public final DoubleSupplier temp;
    
    // outputs
    public double voltageRequest;
    

    public FlywheelSpark() {
        flywheel = new SparkMax(FlywheelConstants.FlywheelCANID, MotorType.kBrushless);
        var config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(FlywheelConstants.reduction);
        config.closedLoop.positionWrappingEnabled(false);
        config.inverted(FlywheelConstants.inverted)
              .idleMode(FlywheelConstants.idleMode)
              .smartCurrentLimit(FlywheelConstants.StallLimit,FlywheelConstants.FreeLimit);
        tryUntilOk(flywheel, 5, () -> flywheel.configure(config,ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        flywheelencoder = flywheel.getEncoder();
        tryUntilOk(flywheel, 5, () -> flywheelencoder.setPosition(0.0));
        position = () -> flywheelencoder.getPosition();   
        velocity = () -> flywheelencoder.getVelocity();
        appliedVolts = () -> flywheel.getAppliedOutput();
        temp = () -> flywheel.getMotorTemperature();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.data = new FlywheelIOData(
            flywheel.getLastError() != REVLibError.kOk,
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
    public void stop() {
        flywheel.stopMotor();
    }

    


}
