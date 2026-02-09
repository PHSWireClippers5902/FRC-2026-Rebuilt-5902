package org.frc5902.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import org.frc5902.robot.Constants.QuestConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class QuestThread {
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Supplier<Pose3d>> genericPoses = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();
    private final List<Queue<Pose3d>> genericPoseQueues = new ArrayList<>();

    private static QuestThread instance = null;
    private Notifier notifier = new Notifier(this::run);

    public static QuestThread getInstance() {
        if (instance == null) {
            instance = new QuestThread();
        }
        return instance;
    }

    private QuestThread() {
        notifier.setName("QuestThread");
    }

    public void start() {
        if (timestampQueues.size() > 0) {
            notifier.startPeriodic(1.0 / QuestConstants.questFrequency);
        }
    }

    /** Registers a generic signal to be read from the thread. */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        QuestSubsystem.questLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            QuestSubsystem.questLock.unlock();
        }
        return queue;
    }

    /** Registers a Pose3d signal to be read from the thread. */
    public Queue<Pose3d> registerSignal(Supplier<Pose3d> pose) {
        Queue<Pose3d> queue = new ArrayBlockingQueue<>(20);
        QuestSubsystem.questLock.lock();
        try {
            genericPoses.add(pose);
            genericPoseQueues.add(queue);
        } finally {
            QuestSubsystem.questLock.unlock();
        }
        return queue;
    }

    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        QuestSubsystem.questLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            QuestSubsystem.questLock.unlock();
        }
        return queue;
    }

    private void run() {
        // Save new data to queues
        QuestSubsystem.questLock.lock();
        try {
            // Get sample timestamp
            double timestamp = RobotController.getFPGATime() / 1e6;
            // If valid, add values to queues
            for (int i = 0; i < genericSignals.size(); i++) {
                genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
            }
            for (int i = 0; i < timestampQueues.size(); i++) {
                timestampQueues.get(i).offer(timestamp);
            }
            for (int i = 0; i < genericPoseQueues.size(); i++) {
                genericPoseQueues.get(i).offer(genericPoses.get(i).get());
            }
        } finally {
            QuestSubsystem.questLock.unlock();
        }
    }
}
