/**
 * IF
 * @blame Daniel Sabalakov
 */
package org.frc5902.robot.util.fieldbased;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.ToString;
import org.frc5902.robot.state.RobotState;

@ToString
public class FieldCommand extends Command {

    public Debouncer debounceFinish = new Debouncer(0.5, DebounceType.kBoth);
    public Shape2d shape;
    public Command whatToRun;

    public FieldCommand(Shape2d encapsulatingShape, Command whatToRun) {
        this.shape = encapsulatingShape;
    }
    // let the command scheduler do its thing
    @Override
    public void initialize() {
        whatToRun.initialize();
    }

    @Override
    public void execute() {
        whatToRun.execute();
    }

    @Override
    public void end(boolean interrupted) {
        whatToRun.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return debounceFinish.calculate(
                shape.insideShape(RobotState.getInstance().getEstimatedPose().getTranslation()));
    }
}
