package org.frc5902.robot.subsystems.drive.modules;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@Getter
@Setter
@ToString
public class ModuleConfiguration {
    public int DrivingID = 0;
    public int TurningID = 0;
    public int TurningEncoderID = 0;
    public boolean DrivingMotorInverted = false;
    public boolean TurningMotorInverted = true;
    public boolean TurnSensorInvert = true;
    public Translation2d ModuleOffset = new Translation2d(0, 0);
    public double MagnetOffset = 0;

    @Builder
    public ModuleConfiguration(
            int DrivingID,
            int TurningID,
            int TurningEncoderID,
            boolean DrivingMotorInverted,
            boolean TurningMotorInverted,
            boolean TurnSensorInvert,
            Translation2d ModuleOffset,
            double MagnetOffset) {
        this.DrivingID = DrivingID;
        this.TurningID = TurningID;
        this.TurningEncoderID = TurningEncoderID;
        this.DrivingMotorInverted = DrivingMotorInverted;
        this.TurningMotorInverted = TurningMotorInverted;
        this.TurnSensorInvert = TurnSensorInvert;
        this.ModuleOffset = ModuleOffset;
        this.MagnetOffset = MagnetOffset;
    }

    public static ModuleConfigurationBuilder builder() {
        return new ModuleConfigurationBuilder();
    }
}
