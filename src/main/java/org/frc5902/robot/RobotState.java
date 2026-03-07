package org.frc5902.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.frc5902.robot.subsystems.drive.DriveConstants.ModuleConfigurations;
import org.frc5902.robot.util.buildutil.GeoUtil;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.NoSuchElementException;
import java.util.Optional;

@ExtensionMethod({GeoUtil.class})
public class RobotState {
    // Pose Estimation
    private static final double poseBufferSizeSec = 2.0;
    // trust up to 3 thousands of a meter, three thousandths of a meter, and two thousandths of a meter.
    private static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));
    private static final Matrix<N3, N1> questStdDevs = new Matrix<>(VecBuilder.fill(0.02, 0.02, 0.035));

    private static RobotState instance;
    // create static instance (only once... only once)
    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    @Getter
    @AutoLogOutput
    private Pose2d odometryPose = Pose2d.kZero;

    @Getter
    @AutoLogOutput
    private Pose2d estimatedPose = Pose2d.kZero;

    // to create past estimates
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    private final TimeInterpolatableBuffer<Rotation3d> rotationBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

    private final SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    // by default gyro will begin at zero.
    private Rotation2d gyroOffset = Rotation2d.kZero;

    @Getter
    @AutoLogOutput(key = "RobotState/RobotVelocity")
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    @Getter
    @Setter
    private Rotation2d pitch = Rotation2d.kZero;

    @Getter
    @Setter
    private Rotation2d roll = Rotation2d.kZero;

    private RobotState() {
        for (int i = 0; i < 3; ++i) {
            // square odometry stdevs
            // needs to increment by ++i (pre-adding)
            qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
        }
        kinematics = new SwerveDriveKinematics(ModuleConfigurations.moduleTranslations);
    }

    // Gyro offset is rotation that maps old rotation to new frame of rotation
    public void resetPose(Pose2d pose) {
        gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
        estimatedPose = pose;
        odometryPose = pose;
        poseBuffer.clear();
    }

    public void addOdometryObservation(OdometryObservation observation) {
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        Pose2d lastOdometryPose = odometryPose;
        odometryPose = odometryPose.exp(twist);
        // if gyroscope is connected
        observation.yaw.ifPresent(gyroAngle -> {
            // add offset to angle
            Rotation2d angle = gyroAngle.plus(gyroOffset);
            odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        });

        // pose buffer will now buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);
        // Calculate difference from last odo pose and add onto the pose estimate...
        if (observation.roll.isPresent()) {
            rotationBuffer.addSample(observation.timestamp(), new Rotation3d());
        }

        Twist2d finalTwist = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(finalTwist);
    }

    public void addVisionObservation(VisionObservation observation) {
        // if its too old, throw it out...
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }

        // allows compiler to assume...
        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) {
            return;
        }

        // sample to odometry pose transform and backwards
        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // calc 3x3 vision matrix (idk why Mecha-Advantage used ++i)
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
        }
        // as Mechanical Advantage wrote (and wise words from them...)
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }

        // calculate difference between estimate and vision pose
        Transform2d transform =
                new Transform2d(estimateAtTime, observation.visionPose().toPose2d());
        // transform by visionK
        var kTimesTransform = visionK.times(VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        Transform2d scaledTransform = new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        // recalc estimate by applying transform to the old estimate
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public void addDriveSpeeds(ChassisSpeeds speeds) {
        robotVelocity = speeds;
    }

    @AutoLogOutput(key = "RobotState/FieldVelocity")
    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
    }

    public Rotation2d getRotation() {
        return estimatedPose.getRotation();
    }

    public void addQuestObservation(QuestObservation observation) {
        // if obso is old to be outside the pose buffer's timespan, skip please!
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }
        // allows compiler to assume...
        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) {
            return;
        }

        Pose2d estimateAtTime = estimatedPose;

        // calc 3x3 vision matrix (idk why Mecha-Advantage used ++i)
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = questStdDevs.get(i, 0) * questStdDevs.get(i, 0);
        }
        // as Mechanical Advantage wrote (and wise words from them...)
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }

        // calculate difference between estimate and vision pose
        Transform2d transform =
                new Transform2d(estimateAtTime, observation.pose().toPose2d());
        // transform by visionK
        var kTimesTransform = visionK.times(VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        Transform2d scaledTransform = new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));
        estimatedPose = estimateAtTime.plus(scaledTransform);
    }

    public record OdometryObservation(
            SwerveModulePosition[] wheelPositions,
            Optional<Rotation2d> roll,
            Optional<Rotation2d> pitch,
            Optional<Rotation2d> yaw,
            double timestamp) {}

    public record VisionObservation(Pose3d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

    public record QuestObservation(Pose3d pose, double timestamp) {}
}
