package frc.robot.triggers;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SPEED_LIMIT_BUTTON;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class LimitSpeedButton extends Command{
    public Dimensionless m_speedLimitAxis = SWERVE.AXIS_MAX_SPEED;
    public Dimensionless m_speedLimitAngular = SWERVE.AXIS_MAX_ANGULAR_RATE;
    
    public boolean m_buttonState = true;

    public void initialize() {
        m_speedLimitAxis = SPEED_LIMIT_BUTTON.SPEED_LIMIT_AXIS;
        m_speedLimitAngular = SPEED_LIMIT_BUTTON.SPEED_LIMIT_ANGULAR;

        Robot.swerve.setTranslationModifier(m_speedLimitAxis);

        m_speedLimitAxis = m_buttonState ? SPEED_LIMIT_BUTTON.SPEED_LIMIT_AXIS : 
            SWERVE.AXIS_MAX_SPEED;

        m_speedLimitAngular = m_buttonState ? SPEED_LIMIT_BUTTON.SPEED_LIMIT_ANGULAR : 
            SWERVE.AXIS_MAX_ANGULAR_RATE;

        m_buttonState = !m_buttonState;

        Robot.swerve.setTranslationModifier(m_speedLimitAxis);
        Robot.swerve.setRotationModifier(m_speedLimitAngular);
    }
}