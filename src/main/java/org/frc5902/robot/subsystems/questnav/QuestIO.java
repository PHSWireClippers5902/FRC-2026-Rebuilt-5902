package org.frc5902.robot.subsystems.questnav;

import org.littletonrobotics.junction.AutoLog;

public class QuestIO {
    @AutoLog
    public class QuestIOInputs {
        public boolean connected = false;

        public double readGyroscopicPosition = 0.0;

        
    }

    public void updateInputs(QuestIOInputs inputs){
        
    }
}
