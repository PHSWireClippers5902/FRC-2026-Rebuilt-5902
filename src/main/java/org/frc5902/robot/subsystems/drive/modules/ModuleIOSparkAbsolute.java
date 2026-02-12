package org.frc5902.robot.subsystems.drive.modules;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import org.frc5902.robot.Constants.*;
import org.frc5902.robot.subsystems.drive.DriveConstants.*;
import org.frc5902.robot.subsystems.drive.SparkOdometryThread;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import static org.frc5902.robot.util.SparkUtil.*;

public class ModuleIOSparkAbsolute implements ModuleIO {
    private final Rotation2d zeroRotation;

    private final SparkBase driveSpark;
    private final SparkBase turnSpark;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final CANcoder cancoder;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;
    // timestamp inputs
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<Angle> turnAbsolutePosition;

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final ModuleConfiguration MODULE_INFORMATION;

    public ModuleIOSparkAbsolute(int module) {
        MODULE_INFORMATION = switch (module) {
            case 0 -> ModuleConfigurations.FrontLeftModule;
            case 1 -> ModuleConfigurations.FrontRightModule;
            case 2 -> ModuleConfigurations.BackLeftModule;
            case 3 -> ModuleConfigurations.BackRightModule;
            default -> ModuleConfigurations.FrontLeftModule;
        };
        System.out.println("Module " + MODULE_INFORMATION.toString());
        System.out.println("Module " + MODULE_INFORMATION.DrivingID);

        // cancoder configurations
        cancoder = new CANcoder(MODULE_INFORMATION.TurningEncoderID);
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = MODULE_INFORMATION.getMagnetOffset();
        // cancoderConfig.MagnetSensor.MagnetOffset = 0;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);
        turnAbsolutePosition = cancoder.getPosition();
        System.out.println("Turn Absolute Position; With Mag Offsets (if aligned, should be zero), Module " + module
                + " is: " + turnAbsolutePosition.getValueAsDouble());

        // zeroRotation = Rotation2d.fromRotations(MODULE_INFORMATION.getMagnetOffset());
        zeroRotation = Rotation2d.kZero;
        driveSpark = new SparkMax(MODULE_INFORMATION.DrivingID, MotorType.kBrushless);
        turnSpark = new SparkMax(MODULE_INFORMATION.TurningID, MotorType.kBrushless);

        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();

        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        var driveConfig = new SparkMaxConfig();
        driveConfig
                .inverted(MODULE_INFORMATION.DrivingMotorInverted)
                .idleMode(DriveMotorConstants.driveIdleMode)
                .smartCurrentLimit(DriveMotorConstants.driveCurrentLimit)
                .voltageCompensation(DriveMotorConstants.driveVoltageCompensation);
        driveConfig
                .encoder
                .positionConversionFactor(DriveMotorConstants.drivePositionConversionFactor)
                .velocityConversionFactor(DriveMotorConstants.driveVelocityConversionFactor);
        driveConfig
                .closedLoop
                .feedbackSensor(DriveMotorConstants.driveFeedbackSensor)
                .pid(
                        DriveMotorConstants.driveClosedLoop.getProportional(),
                        DriveMotorConstants.driveClosedLoop.getIntegral(),
                        DriveMotorConstants.driveClosedLoop.getDeriviative());
        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / OdometryConstants.odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(RobotConstants.periodMs)
                .appliedOutputPeriodMs(RobotConstants.periodMs)
                .busVoltagePeriodMs(RobotConstants.periodMs)
                .outputCurrentPeriodMs(RobotConstants.periodMs);
        tryUntilOk(
                driveSpark,
                5,
                () -> driveSpark.configure(
                        driveConfig, DriveMotorConstants.driveResetMode, DriveMotorConstants.drivePersistMode));
        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0));

        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(MODULE_INFORMATION.TurningMotorInverted)
                .idleMode(TurnMotorConstants.turnIdleMode)
                .smartCurrentLimit(TurnMotorConstants.turningCurrentLimit)
                .voltageCompensation(TurnMotorConstants.turnVoltageCompensation);
        turnConfig
                .encoder
                .positionConversionFactor(TurnMotorConstants.turnPositionConversionFactor)
                .velocityConversionFactor(TurnMotorConstants.turnVelocityConversionFactor);
        turnConfig
                .closedLoop
                .feedbackSensor(TurnMotorConstants.turningFeedbackSensor)
                .positionWrappingEnabled(TurnMotorConstants.turnPositionWrappingEnabled)
                .positionWrappingInputRange(TurnMotorConstants.turnPIDMinInput, TurnMotorConstants.turnPIDMaxInput)
                .pid(
                        TurnMotorConstants.turnClosedLoop.getProportional(),
                        TurnMotorConstants.turnClosedLoop.getIntegral(),
                        TurnMotorConstants.turnClosedLoop.getDeriviative());
        System.out.println(TurnMotorConstants.turnClosedLoop);
        turnConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / OdometryConstants.odometryFrequency))
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(RobotConstants.periodMs)
                .appliedOutputPeriodMs(RobotConstants.periodMs)
                .busVoltagePeriodMs(RobotConstants.periodMs)
                .outputCurrentPeriodMs(RobotConstants.periodMs);

        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(
                        turnConfig, TurnMotorConstants.turnResetMode, TurnMotorConstants.turnPersistMode));
        tryUntilOk(
                turnSpark,
                5,
                () -> turnEncoder.setPosition(turnAbsolutePosition.getValueAsDouble()
                        * TurnMotorConstants.turnPositionAbsoluteConversionFactor));

        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // refresh cancoder signal
        var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);
        inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePosition =
                Rotation2d.fromRadians(Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                                .getRadians()
                        % (2 * Math.PI));
        // if you are confused on what "->" are, read https://www.w3schools.com/java/java_lambda.asp
        sparkStickyFault = false;
        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRadians = value);
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadiansPerSecond = value);
        // gets output and multiplies it by voltage
        ifOk(
                driveSpark,
                new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        // amperage update
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // turn inputs
        ifOk(
                turnSpark,
                turnEncoder::getPosition,
                (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadiansPerSecond = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // odometry
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRadians =
                drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        // clear queues
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    // sets drive motor at the specific open loop value
    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    // set turn open loop at specified open loop value
    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    // set drive motor at velocity
    @Override
    public void setDriveVelocity(double velocityRadiansPerSecond) {
        // lets calculate feed forward volts
        double feedForwardVolts = DriveMotorConstants.kS * Math.signum(velocityRadiansPerSecond)
                + DriveMotorConstants.kV * velocityRadiansPerSecond;
        driveController.setSetpoint(
                velocityRadiansPerSecond,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                feedForwardVolts,
                ArbFFUnits.kVoltage);
    }

    // sets motor specific position
    @Override
    public void setTurnPosition(Rotation2d rotation) {
        // calculate optimal position
        double setpoint = MathUtil.inputModulus(
                rotation.plus(zeroRotation).getRadians(),
                TurnMotorConstants.turnPIDMinInput,
                TurnMotorConstants.turnPIDMaxInput);
        turnController.setSetpoint(setpoint, ControlType.kPosition);
    }

    @Override
    public void setRelativeToAbsoluteValues() {
        tryUntilOk(
                turnSpark,
                5,
                () -> turnEncoder.setPosition(turnAbsolutePosition.getValueAsDouble()
                        * TurnMotorConstants.turnPositionAbsoluteConversionFactor));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        IdleMode mode = enabled ? IdleMode.kBrake : IdleMode.kCoast;

        tryUntilOk(
                driveSpark,
                5,
                () -> turnSpark.configureAsync(
                        new SparkMaxConfig().idleMode(mode),
                        DriveMotorConstants.driveResetMode,
                        DriveMotorConstants.drivePersistMode));

        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configureAsync(
                        new SparkMaxConfig().idleMode(mode),
                        TurnMotorConstants.turnResetMode,
                        TurnMotorConstants.turnPersistMode));
    }
}
