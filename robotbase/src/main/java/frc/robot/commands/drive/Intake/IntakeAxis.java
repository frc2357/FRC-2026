package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class IntakeStopextends Command {

    private AxisInterface m_axis;

    public IntakeAxis(AxisInterface axis) {
        m_axis = axis;
        addRequirements(Robot.Intake);
    }

    @Override
    public void execute() {
        double axisValue = m_axis.getValue();
        Robot.Intake.setAxisSpeed(axisValue);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        robot.Intake.stop();
    }
}
