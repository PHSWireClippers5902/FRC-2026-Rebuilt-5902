package org.frc5902.robot.subsystems.drive.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5902.robot.Constants.*;
import org.frc5902.robot.subsystems.drive.DriveConstants.*;
import org.frc5902.robot.subsystems.drive.SparkOdometryThread;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import static org.frc5902.robot.util.SparkUtil.*;

public class ModuleIOTalonAbsolute
        implements ModuleIO { // <----- PLEASE IMPLEMENT MODULEIO..... WITHOUT THIS YOU CANNOT ACCESS OTHER CLASSES

    private final Rotation2d zeroRotation;

    private final SparkBase driveSpark;
    private final WPI_TalonSRX turnTalons;

    private final RelativeEncoder driveEncoder;

    //     private final SparkClosedLoopController turnController;
    private final SparkClosedLoopController driveController;

    // private final Queue<Double> turnPositionQueue;
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private ModuleConfiguration MODULE_INFORMATION;

    public ModuleIOTalonAbsolute(int module) {
        System.out.println("Module " + module);
        MODULE_INFORMATION = switch (module) {
            case 0 -> ModuleConfigurations.FrontLeftModule;
            case 1 -> ModuleConfigurations.FrontRightModule;
            case 2 -> ModuleConfigurations.BackLeftModule;
            case 3 -> ModuleConfigurations.BackRightModule;
            default -> ModuleConfigurations.FrontLeftModule;
        };

        // zeroRotation = MODULE_INFORMATION.ZeroRotation;
        // if relative, there is NO zero rotation
        zeroRotation = Rotation2d.kZero; // is this right bcz it is actually absolute?

        driveSpark = new SparkMax(MODULE_INFORMATION.DrivingID, MotorType.kBrushless);
        turnTalons = new WPI_TalonSRX(MODULE_INFORMATION.TurningID);
        driveController = driveSpark.getClosedLoopController();

        driveEncoder = driveSpark.getEncoder();

        turnTalons.configFactoryDefault();
        turnTalons.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        turnTalons.setSensorPhase(true); // come back later
        turnTalons.setInverted(MODULE_INFORMATION.TurningMotorInverted);
        turnTalons.configNominalOutputForward(0);
        turnTalons.configNominalOutputReverse(0);
        turnTalons.configPeakOutputForward(1);
        turnTalons.configPeakOutputReverse(1);

        turnTalons.config_kP(0, 0.5);
        turnTalons.config_kI(0, 0);
        turnTalons.config_kD(0, 1);

        turnTalons.configSelectedFeedbackCoefficient(1 / 4096, 0, 0);
        int absolutePosition =
                turnTalons.getSensorCollection().getPulseWidthPosition() % 4096; // Mask out to 12-bit value
        int absoluteChange = 0; // DriveConstants.AbsoluteChange; Replace when lil d can show me
        turnTalons.setSelectedSensorPosition(absolutePosition - absoluteChange, 0, 0);

        turnTalons.configVoltageCompSaturation(TurnMotorConstants.turnVoltageCompensation);
        turnTalons.configContinuousCurrentLimit(
                TurnMotorConstants.turningCurrentLimit); // @todo add a peak and continuous motor current limit
        turnTalons.setNeutralMode(NeutralMode.Brake);

        System.out.println("DEFAULT FRONT LEFT MODULE INFORMAITON" + ModuleConfigurations.FrontLeftModule.toString());
        System.out.println("Module " + MODULE_INFORMATION.toString());
        System.out.println("Module " + MODULE_INFORMATION.DrivingID);
        System.out.println("Module" + MODULE_INFORMATION.DrivingID
                + absolutePosition); // prints out position for manual alignement

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

        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
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
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // odometry
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRadians =
                drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        // inputs.odometryTurnPositions = turnPositionQueue.stream()    //ask lil D abt drive position que
        //         .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
        //         .toArray(Rotation2d[]::new);
        // clear queues
        timestampQueue.clear();
        drivePositionQueue.clear();
        // turnPositionQueue.clear();
    }

    // sets drive motor at the specific open loop value
    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    // set turn open loop at specified open loop value
    @Override
    public void setTurnOpenLoop(double output) {
        turnTalons.setVoltage(output);
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
        double setpoint = MathUtil.inputModulus(
                rotation.plus(zeroRotation).getRadians(),
                TurnMotorConstants.turnPIDMinInput,
                TurnMotorConstants.turnPIDMaxInput);
        turnTalons.set(ControlMode.Position, setpoint); // turn is talon not 550 Check me lil D
    }
}
