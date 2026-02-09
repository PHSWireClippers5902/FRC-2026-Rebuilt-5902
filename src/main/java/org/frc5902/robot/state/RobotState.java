package org.frc5902.robot.state;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.frc5902.robot.FieldConstants;
import org.frc5902.robot.subsystems.drive.DriveConstants.ModuleConfigurations;
import org.frc5902.robot.subsystems.vision.VisionConstants;
import org.frc5902.robot.util.GeoUtil;
import org.frc5902.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;

@ExtensionMethod({GeoUtil.class})
public class RobotState {
    // Must be less than 2.0
    private static final LoggedTunableNumber txTyObservationStaleSecs =
            new LoggedTunableNumber("RobotState/TxTyObservationStaleSeconds", 0.2);
    private static final LoggedTunableNumber minDistanceTagPoseBlend =
            new LoggedTunableNumber("RobotState/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
    private static final LoggedTunableNumber maxDistanceTagPoseBlend =
            new LoggedTunableNumber("RobotState/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));

    // Pose Estimation
    private static final double poseBufferSizeSec = 2.0;
    // trust up to 3 thousands of a meter, three thousandths of a meter, and two thousandths of a meter.
    private static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));
    private static final Matrix<N3, N1> questStdDevs = new Matrix<>(VecBuilder.fill(0.02, 0.02, 0.035));
    private static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();
    // initialize tag poses (in two dimensions)
    static {
        // start at 1 because hashmaps are stupid and begin at 1
        for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
            tagPoses2d.put(
                    i,
                    FieldConstants.defaultAprilTagType
                            .getLayout()
                            .getTagPose(i)
                            .map(Pose3d::toPose2d)
                            .orElse(Pose2d.kZero));
        }
    }

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
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
    private final SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    // by default gyro will begin at zero.
    private Rotation2d gyroOffset = Rotation2d.kZero;

    private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();

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

        for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
            txTyPoses.put(i, new TxTyPoseRecord(Pose2d.kZero, Double.POSITIVE_INFINITY, -1.0));
        }
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
        observation.gyroAngle.ifPresent(gyroAngle -> {
            // add offset to angle
            Rotation2d angle = gyroAngle.plus(gyroAngle);
            odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        });

        // pose buffer will now buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);
        // Calculate difference from last odo pose and add onto the pose estimate...
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
        for (int i = 1; i < 4; ++i) {
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
        Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
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

    public void addTxTyObservation(TxTyObservaction observation) {
        // make sure this is the newest key
        if (txTyPoses.containsKey(observation.tagId())
                && txTyPoses.get(observation.tagId()).timestamp() >= observation.timestamp()) {
            return;
        }

        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) return;

        Rotation2d robotRotation = estimatedPose
                .transformBy(new Transform2d(odometryPose, sample.get()))
                .getRotation();

        // average tX and tY
        double tx = 0.0;
        double ty = 0.0;
        for (int i = 0; i < 4; i++) {
            tx += observation.tx()[i];
            ty += observation.ty()[i];
        }
        tx /= 4.0;
        ty /= 4.0;

        // TODO IMPLEMENT GET CAMERA POSE
        Pose3d cameraPose = VisionConstants.cameras[observation.camera()].pose().get();
        // use 3D distance and angles to find robo pose
        Translation2d camToTagTranslation = new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, -tx))
                .transformBy(new Transform3d(new Translation3d(observation.distance(), 0, 0), Rotation3d.kZero))
                .getTranslation()
                .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
                .toTranslation2d();
        Rotation2d camToTagRotation =
                robotRotation.plus(cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));
        var tagPose2d = tagPoses2d.get(observation.tagId());
        if (tagPose2d == null) return;
        Translation2d fieldToCameraTranslation = new Pose2d(
                        tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(GeoUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
                .getTranslation();
        Pose2d robotPose = new Pose2d(
                        fieldToCameraTranslation,
                        robotRotation.plus(cameraPose.toPose2d().getRotation()))
                .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));

        robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

        txTyPoses.put(
                observation.tagId(),
                new TxTyPoseRecord(robotPose, camToTagTranslation.getNorm(), observation.timestamp()));
    }

    public void addDriveSpeeds(ChassisSpeeds speeds) {
        robotVelocity = speeds;
    }

    @AutoLogOutput(key = "RobotState/FieldVelocity")
    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
    }

    public Optional<Pose2d> getTxTyPose(int tagId) {
        if (!txTyPoses.containsKey(tagId)) {
            return Optional.empty();
        }
        var data = txTyPoses.get(tagId);
        // if stale
        if (Timer.getTimestamp() - data.timestamp() >= txTyObservationStaleSecs.get()) {
            return Optional.empty();
        }

        var sample = poseBuffer.getSample(data.timestamp());
        // latency comp
        return sample.map(pose2d -> data.pose().plus(new Transform2d(pose2d, odometryPose)));
    }

    public Rotation2d getRotation() {
        return estimatedPose.getRotation();
    }

    public void periodic() {
        Pose2d[] tagPoses = new Pose2d[FieldConstants.aprilTagCount + 1];
        for (int i = 0; i < FieldConstants.aprilTagCount + 1; i++) {
            tagPoses[i] = getTxTyPose(i).orElse(Pose2d.kZero);
        }
        Logger.recordOutput("RobotState/TxTyPoses", tagPoses);
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
        for (int i = 1; i < 4; ++i) {
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
            SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
    // what do these do? We'll see...
    public record TxTyObservaction(
            int tagId, int camera, double[] tx, double[] ty, double distance, double timestamp) {}

    public record TxTyPoseRecord(Pose2d pose, double distance, double timestamp) {}

    public record QuestObservation(Pose3d pose, Twist3d twist, boolean tracking, double timestamp) {}
}
