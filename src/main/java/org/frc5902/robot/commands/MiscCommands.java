package org.frc5902.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.frc5902.robot.FieldConstants;
import org.frc5902.robot.commands.drive.DriveCommands;
import org.frc5902.robot.containers.CompRobotContainer;
import org.frc5902.robot.subsystems.drive.Drive;
import org.frc5902.robot.subsystems.superstructure.Superstructure;
import org.frc5902.robot.subsystems.superstructure.SuperstructureActions;

public class MiscCommands {
    public static Command PoseResetCommand(CompRobotContainer crc) {
        return Commands.runOnce(() -> {
                    crc.resetInitialPose();
                })
                .withTimeout(0.1);
    }



    public static Command LaunchSequence(Drive drive, Superstructure superstructure) {
        return Commands.parallel(
            DriveCommands.pointAtPoseOSCILLATE(drive, FieldConstants.BLUE_HUB_LOCATION),
            Commands.sequence(
                Superstructure.AutonomousSuperstructureGoalCommand(SuperstructureActions.READY_LAUNCHER_STUPID, 1, superstructure),
                Superstructure.AutonomousSuperstructureGoalCommand(SuperstructureActions.LAUNCH_STUPID, 3, superstructure)
            )
        );
    }
}
