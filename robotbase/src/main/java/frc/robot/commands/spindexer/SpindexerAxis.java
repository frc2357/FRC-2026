package frc.robot.commands.spindexer;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class SpindexerAxis extends Command {

    private Supplier<Dimensionless> m_axis;

    public SpindexerAxis(Supplier<Dimensionless> axis) {
        m_axis = axis;
        addRequirements(Robot.spindexer);
    }

    @Override
    public void execute() {
        Dimensionless axisValue = m_axis.get();
        Robot.spindexer.setAxisSpeed(axisValue);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.spindexer.stop();
    }
}
