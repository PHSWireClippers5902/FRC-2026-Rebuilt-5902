package org.frc5902.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5902.robot.subsystems.compbot.superstructure.Superstructure;
import org.frc5902.robot.subsystems.compbot.superstructure.SuperstructureActions;
import org.frc5902.robot.subsystems.drive.Drive;

import java.util.function.Supplier;

public class AutoPlease {

    public static Command extendAndMoveAuto(Supplier<Drive> drive, Supplier<Superstructure> supersupplier) {
        return Commands.sequence(
                Commands.parallel(
                        Commands.runOnce(() -> supersupplier.get().addCommandToScheduler(SuperstructureActions.STOW))
                                .withTimeout(1),
                        Commands.run(() -> drive.get().runVelocity(new ChassisSpeeds(0.3, 0, 0)), drive.get())
                                .withTimeout(1)),
                Commands.runOnce(() -> supersupplier.get().removeCommandFromScheduler(SuperstructureActions.STOW))
                        .withTimeout(5));
    }



    public static Command dumbRightAuto(Supplier<Drive> drive, Supplier<Superstructure> supersupplier) {
        Drive d = drive.get();
        Superstructure s = supersupplier.get();
        return Commands.sequence(
                Commands.parallel(
                        Commands.run(() -> drive.get().runVelocity(new ChassisSpeeds(0.3, 0, 0.0)), drive.get())
                                .withTimeout(1),
                        Commands.run(() -> s.addCommandToScheduler(SuperstructureActions.READY_LAUNCHER_STUPID).withTimeout(3))),
                Commands.runOnce(() -> supersupplier.get().removeCommandFromScheduler(SuperstructureActions.STOW))
                        .withTimeout(5)

        );

    }




}
