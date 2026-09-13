package frc.robot.triggers;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.SPEED_LIMIT_BUTTON;
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.Shooter;
import frc.robot.Robot;
import frc.robot.commands.intakepivot.LimitShooter;
import yams.mechanisms.config.FlyWheelConfig;


public class LimitSpeedButton extends Command{
    public Dimensionless m_speedLimitAxis = SWERVE.AXIS_MAX_SPEED;
    public Dimensionless m_speedLimitAngular = SWERVE.AXIS_MAX_ANGULAR_RATE;

    public FlyWheelConfig m_flyWheelConfig;
    public Shooter m_shooter;

    public Command currentCommand;

    public AngularVelocity stopSpeed = Constants.AUTO.AUTO_SHOOTER_STOP;
    public AngularVelocity fullSpeed = Constants.AUTO.AUTO_SHOOTER_IDLE;
    
    public boolean m_buttonState = true;

    public String m_buttonStateString = "Speed: Full";

    public LimitSpeedButton(Shooter shooter) {
        m_shooter = shooter;
        currentCommand = m_shooter.getCurrentCommand();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        currentCommand = m_shooter.getCurrentCommand();

        m_speedLimitAxis = SPEED_LIMIT_BUTTON.SPEED_LIMIT_AXIS;
        m_speedLimitAngular = SPEED_LIMIT_BUTTON.SPEED_LIMIT_ANGULAR;

        Robot.swerve.setTranslationModifier(m_speedLimitAxis);

        m_speedLimitAxis = m_buttonState ? SPEED_LIMIT_BUTTON.SPEED_LIMIT_AXIS : 
            SWERVE.AXIS_MAX_SPEED;

        m_speedLimitAngular = m_buttonState ? SPEED_LIMIT_BUTTON.SPEED_LIMIT_ANGULAR : 
            SWERVE.AXIS_MAX_ANGULAR_RATE;

        m_buttonState = !m_buttonState;

        if (m_buttonState == false) {
            m_buttonStateString = "Speed: Limited";

            new LimitShooter(false);

            CommandScheduler.getInstance().cancel(currentCommand);
            m_shooter.removeDefaultCommand();
            m_shooter.setDefaultCommand(m_shooter.setStopVelocity());
        }
        else {
            m_buttonStateString = "Speed: Full";

            new LimitShooter(true);

            CommandScheduler.getInstance().cancel(currentCommand);
            m_shooter.removeDefaultCommand();
            m_shooter.setDefaultCommand(m_shooter.setIdleVelocity());
        }

        SmartDashboard.putString("ButtonState", m_buttonStateString);

        Robot.swerve.setTranslationModifier(m_speedLimitAxis);
        Robot.swerve.setRotationModifier(m_speedLimitAngular);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}