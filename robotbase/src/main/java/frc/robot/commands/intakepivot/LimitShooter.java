package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LimitShooter extends Command{
    public static boolean m_isShooterLimited;

    public LimitShooter(boolean isShooterLimited) {
        m_isShooterLimited = isShooterLimited;
    }

    @Override
    public void initialize() {
        
    }
}
