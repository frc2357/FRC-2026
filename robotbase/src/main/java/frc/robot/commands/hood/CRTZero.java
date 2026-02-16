package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.HOOD;

public class CRTZero extends Command {
    private boolean m_crtSucceeded = false;

    public CRTZero() {
        addRequirements(Robot.hood);
    }

    @Override
    public void execute() {
        if (Robot.hood.getMotorVelocity().lte(HOOD.CRT_STATIONARY_TOLERANCE)) {
            m_crtSucceeded = Robot.hood.zero();
        }
    }

    @Override
    public boolean isFinished() {
        return m_crtSucceeded;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}