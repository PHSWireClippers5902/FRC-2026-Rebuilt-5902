package org.frc5902.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Setter;

import org.frc5902.robot.Constants;
import org.frc5902.robot.Constants.PathPlannerConstants;
import org.frc5902.robot.Constants.RobotConstants;
import org.frc5902.robot.Constants.RobotConstants.Mode;
import org.frc5902.robot.state.RobotState;
import org.frc5902.robot.subsystems.drive.DriveConstants.ModuleConfigurations;
import org.frc5902.robot.subsystems.drive.gyro.GyroIO;
import org.frc5902.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import org.frc5902.robot.subsystems.drive.modules.Module;
import org.frc5902.robot.subsystems.drive.modules.ModuleIO;
import org.frc5902.robot.util.LocalADStarAK;
import org.frc5902.robot.util.LoggedTunableNumber;
import org.frc5902.robot.util.swerve.SwerveSetpoint;
import org.frc5902.robot.util.swerve.SwerveSetpointGenerator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Volts;

public class Drive extends SubsystemBase {
    public static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final Module[] modules = new Module[4]; // Front Left, Front Right, Back Left, Back Right
    private final Debouncer gyroConnectedDebouncer = 
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private static final LoggedTunableNumber coastWaitTime = new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
    private static final LoggedTunableNumber coastMetersPerSecondThreshold =
            new LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", .05);
    
    @AutoLogOutput
    private boolean velocityMode = false;

    @AutoLogOutput
    private boolean brakeModeEnabled = true;

    private final Timer lastMovementTimer = new Timer();
    
    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(ModuleConfigurations.moduleTranslations);

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[] {
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    });
    private final SwerveSetpointGenerator swerveSetpointGenerator;
    
    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);
        lastMovementTimer.start();
        // todo add setBrakeMode(true);

        swerveSetpointGenerator = new SwerveSetpointGenerator(kinematics, ModuleConfigurations.moduleTranslations);

        SparkOdometryThread.getInstance().start();
    }

    public enum CoastRequest {
        AUTOMATIC,
        ALWAYS_BRAKE,
        ALWAYS_COAST
    }
    @Setter @AutoLogOutput private CoastRequest coastRequest = CoastRequest.ALWAYS_BRAKE;


    @Override
    public void periodic() {
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        if (DriverStation.isDisabled()) {
            Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        

        double[] sampleTimestamps = RobotConstants.currentMode == Mode.SIM ? 
                                        new double[] {Timer.getTimestamp()} :
                                        gyroInputs.odometryYawTimestamps;
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
            }
            RobotState.getInstance()
                    .addOdometryObservation(new RobotState.OdometryObservation(
                            modulePositions,
                            Optional.ofNullable(gyroInputs.data.connected() ? gyroInputs.odometryYawPositions[i] : null),
                            sampleTimestamps[i]));
            // log 3d robot
            Logger.recordOutput("RobotState/EstimatedPose3d", 
                new Pose3d(RobotState.getInstance().getEstimatedPose())
                    .exp(
                        new Twist3d(
                            0,
                            0,
                        )
                    ) 
            );
        }

        RobotState.getInstance().addDriveSpeeds(getChassisSpeeds());

        gyroDisconnectedAlert.set(!gyroInputs.data.connected() && RobotConstants.currentMode != Mode.SIM);
    }

    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, RobotConstants.MAX_SPEED_METERS_PER_SECOND);

        // Log unoptimize Setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // send to mods
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    // stop in x orientation
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = ModuleConfigurations.moduleTranslations[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    // gets quasistatic test in direction (for constant determination)
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        // run 0.0 characterization THEN go to direction
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    // runs dynamic test in direction
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    // get module states and log
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    // get module positions
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    // get chassis speeds autologged
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    // get each position of each module in radians
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharactierizationPosition();
        }
        return values;
    }

    // get feedforward velocity (average velocity), radians per second
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    // get measured odometry pose
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    // add new vision measurement????
    // TODO MOVE TO OWN SUBSYSTEM

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timeStampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timeStampSeconds, visionMeasurementStdDevs);
    }

    // get max linear speed
    public double getMaxLinearSpeedMetersPerSecond() {
        return RobotConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    // TODO IMPLEMENT
    public double getMaxAngularSpeedRadiansPerSecond() {
        return RobotConstants.MAX_SPEED_METERS_PER_SECOND / 1;
    }

    // reset gyro fast commit
    public void resetGyroscope() {
        gyroIO.resetGyro();
    }

    // reset Absolute Positions
    public void resetSwerveAbsolutePositions() {
        for (int i = 0; i < 4; i++) {
            modules[i].resetSwerveAbsolutePositions();
        }
    }
}
