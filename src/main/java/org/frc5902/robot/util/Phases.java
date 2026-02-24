package org.frc5902.robot.util;

import java.util.function.DoubleSupplier;

import org.frc5902.robot.Constants.RobotConstants;

import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@ToString
public class Phases {
    private Phase phase = Phase.AUTO;
    private DoubleSupplier dsTime;
    // assume we lost auto unless we update it.
    @Getter @Setter
    private boolean autonomousWin = false;
    @Getter
    private double timeUntilNextPhase = 0.0;
    @Getter
    private boolean realMatch = false;
    // create instance
    private Phases() {
        dsTime = DriverStation::getMatchTime;
        realMatch = DriverStation.isFMSAttached();
    }
    // instance manager
    private static Phases instance = null;
    public static Phases getInstance() {
        if (instance == null) {
            instance = new Phases();
        }
        return instance;
    }

    public int remainingSeconds() {
        return -1;
    }

    public void teleopInit() {
        phase = Phase.UNDEFINED;
    }

    public boolean canScore() {return canScore(0);}

    public boolean canScore(double timeToFlySeconds) {
        updatePhase();
        if (phase != Phase.AUTO){
            if (phase.allowedToScore && timeUntilNextPhase - timeToFlySeconds < 0){
                return false;
            }
            if (!phase.allowedToScore && timeUntilNextPhase - timeToFlySeconds < 0){
                // you are allowed to shoot if you cannot shoot but it takes enough time to fly
                return true;
            }
            return phase.allowedToScore;
        }
        return true;
    }

    private void updatePhase() {
        double dst = dsTime.getAsDouble();        
        // if the phase is not auto
        if (phase != Phase.AUTO && realMatch) {
            phase = Phase.CANNOT_SCORE;
            if ((autonomousWin) ? dst % 50 <= 25 : dst % 50 >= 25){
                phase = Phase.CAN_SCORE;
            }
            timeUntilNextPhase = 25 - dst % 25;
            // run transition based after the general
            if (dst >= 2 * 60 + 10) {
                phase = Phase.CAN_SCORE;
                timeUntilNextPhase = dst - (2 * 60 + 10);
            }
            if (dst <= 30) {
                phase = Phase.CAN_SCORE;
                timeUntilNextPhase = Double.POSITIVE_INFINITY;
            }
        }
        else if (phase != Phase.AUTO){
            phase = Phase.CANNOT_SCORE;
            // infinitely count down
            if ((autonomousWin) ? dst % 50 <= 25 : dst % 50 >= 25){
                phase = Phase.CAN_SCORE;
            }
            timeUntilNextPhase = 25 - dst % 25;
            if (RobotConstants.SHOOTER_OVERRIDE) {
                phase = Phase.CAN_SCORE;
                timeUntilNextPhase = Double.POSITIVE_INFINITY;
            }
        }
    }

    
    



    public enum Phase {
        CAN_SCORE(true),
        CANNOT_SCORE(false),
        UNDEFINED(true),
        AUTO(true);
        private final boolean allowedToScore;
        private Phase(boolean allowedToScore){
            this.allowedToScore = allowedToScore;
        }
    }
    
    
}
