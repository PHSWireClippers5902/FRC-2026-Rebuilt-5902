package org.frc5902.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frc5902.robot.RobotState;
import org.frc5902.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PointCommands {
    public static Command pointAtTranslation(Drive drive, Translation2d target) {
        return aimAtAngle(drive, () -> target.minus(
                        RobotState.getInstance().getEstimatedPose().getTranslation())
                .getAngle());
    }

    public static Command pointAtPose(Drive drive, Pose2d pose) {
        return aimAtAngle(drive, () -> {
            //     return pose.minus(RobotState.getInstance().getEstimatedPose()).getRotation();
            // return Rotation2d.kZero;
            Logger.recordOutput(
                    "AIM_POSE",
                    pose.getTranslation()
                            .minus(RobotState.getInstance().getEstimatedPose().getTranslation())
                            .getAngle());
            return pose.getTranslation()
                    .minus(RobotState.getInstance().getEstimatedPose().getTranslation())
                    .getAngle();
        });
    }

    public static Command pointAtPoseWhileMoving(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Pose2d pose) {
        return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> {
            Logger.recordOutput(
                    "AIM_POSE",
                    pose.getTranslation()
                            .minus(RobotState.getInstance().getEstimatedPose().getTranslation())
                            .getAngle());
            return pose.getTranslation()
                    .minus(RobotState.getInstance().getEstimatedPose().getTranslation())
                    .getAngle();
        });
    }

    public static Command pointAtPoseOSCILLATE(Drive drive, Pose2d pose) {
        return aimAtAngleOSCLILATE(drive, () -> {
            //     return pose.minus(RobotState.getInstance().getEstimatedPose()).getRotation();
            // return Rotation2d.kZero;
            Logger.recordOutput(
                    "AIM_POSE",
                    pose.getTranslation()
                            .minus(RobotState.getInstance().getEstimatedPose().getTranslation())
                            .getAngle());
            return pose.getTranslation()
                    .minus(RobotState.getInstance().getEstimatedPose().getTranslation())
                    .getAngle();
        });
    }

    public static Command aimAtAngleOSCLILATE(Drive drive, Supplier<Rotation2d> rotationSupplier) {
        // configure (kP, kI, kD, -zoid: max_velocity, max_acceleration)
        ProfiledPIDController angleController =
                new ProfiledPIDController(7.5, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        return Commands.run(
                        () -> {
                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getGyroRotation().getRadians(),
                                    rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? RobotState.getInstance()
                                                    .getRotation()
                                                    .plus(Rotation2d.kPi)
                                            : RobotState.getInstance().getRotation()));
                        },
                        drive)
                .beforeStarting(
                        () -> angleController.reset(drive.getGyroRotation().getRadians()));
    }

    public static Command aimAtAngle(Drive drive, Supplier<Rotation2d> rotationSupplier) {
        // configure (kP, kI, kD, -zoid: max_velocity, max_acceleration)
        ProfiledPIDController angleController =
                new ProfiledPIDController(4, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        return Commands.run(
                        () -> {
                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getGyroRotation().getRadians(),
                                    rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? RobotState.getInstance()
                                                    .getRotation()
                                                    .plus(Rotation2d.kPi)
                                            : RobotState.getInstance().getRotation()));
                        },
                        drive)
                .beforeStarting(
                        () -> angleController.reset(drive.getGyroRotation().getRadians()));
    }

}
