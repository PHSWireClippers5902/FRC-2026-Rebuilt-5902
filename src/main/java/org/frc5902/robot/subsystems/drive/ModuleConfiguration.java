package org.frc5902.robot.subsystems.drive;

import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Builder
@NoArgsConstructor
public class ModuleConfiguration {
    public int ModuleAbsoluteOffset = 0;
    public int DrivingID = 0;
    public int TurningID = 0;
    public ModuleConfiguration(int ModuleAbsoluteOffset, int DrivingID, int TurningID){}
} 
