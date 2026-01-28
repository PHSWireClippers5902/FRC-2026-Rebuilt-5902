package org.frc5902.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.ToString;
import org.frc5902.robot.Constants.*;

@Getter
@Setter
@ToString
public class ModuleConfiguration {
    public int TurningMotorAbsoluteOffset = 0;
    public final Rotation2d ZeroRotation;
    public int DrivingID = 0;
    public int TurningID = 0;
    public boolean DrivingMotorInverted = false;
    public boolean TurningMotorInverted = true;
    public boolean TurnSensorInvert = true;
    public Translation2d ModuleOffset = new Translation2d(0, 0);

    @Builder
    public ModuleConfiguration(
            int TurningMotorAbsoluteOffset,
            int DrivingID,
            int TurningID,
            boolean DrivingMotorInverted,
            boolean TurningMotorInverted,
            boolean TurnSensorInvert,
            Translation2d ModuleOffset) {
        ZeroRotation = Rotation2d.fromRotations(
                (double) TurningMotorAbsoluteOffset / (double) TurnMotorConstants.turnEncoderResolution);
    }

    public static ModuleConfigurationBuilder builder() {
        return new ModuleConfigurationBuilder();
    }
}
