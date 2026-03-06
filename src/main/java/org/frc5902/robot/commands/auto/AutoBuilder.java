package org.frc5902.robot.commands.auto;

import lombok.RequiredArgsConstructor;
import org.frc5902.robot.subsystems.compbot.superstructure.Superstructure;
import org.frc5902.robot.subsystems.drive.Drive;

@RequiredArgsConstructor
public class AutoBuilder {

    private final Drive drive;
    private final Superstructure superstructure;
}
