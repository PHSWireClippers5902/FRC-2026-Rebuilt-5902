package org.frc5902.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5902.robot.containers.CompRobotContainer;

public class MiscCommands {
    public static Command PoseResetCommand(CompRobotContainer crc) {
        return Commands.runOnce(() -> {
                    crc.resetInitialPose();
                })
                .withTimeout(0.1);
    }
}
