package org.frc5902.robot.subsystems.indexer;

import org.frc5902.robot.subsystems.SLAMtake.intake.IntakeIO;
import org.frc5902.robot.subsystems.SLAMtake.intake.IntakeIOInputsAutoLogged;
import org.frc5902.robot.subsystems.SLAMtake.slam.SlamIOInputsAutoLogged;
import org.frc5902.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;
import lombok.Setter;

public class IndexerSystem {
    public IndexerIO indexIO;

    public IndexerIOInputsAutoLogged iIOInputs = new IndexerIOInputsAutoLogged();

    private final Alert IndexerDisconnectedAlert = new Alert("The Indexer Motor has been disconnected. ", AlertType.kError);
    

    @Getter @Setter @AutoLogOutput
    private Goal goal = Goal.STOP;

    public IndexerSystem(IndexerIO io){
        this.indexIO = io;
    }

    public void periodic() {
        indexIO.updateInputs(iIOInputs);

        Logger.processInputs("Indexer/Inputs", iIOInputs);

        IndexerDisconnectedAlert.set(!iIOInputs.data.motorConnected());
        

        switch (goal) {
            
            default: break;
        }
    }



    public enum Goal {
        STOP
    }
}
