package org.frc5902.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSystem extends SubsystemBase{

    private List<Pair<VisionIO, VisionIOInputsAutoLogged>> inputsList = new ArrayList<>();

    
    public VisionSystem(VisionIO... visionIOList) {
        for (VisionIO vio : visionIOList) {
            inputsList.add(Pair.of(vio, new VisionIOInputsAutoLogged()));
        }
    }
 
    
    @Override
    public void periodic() {
        
    }


}
