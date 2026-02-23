package frc.robot.commands.drive;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class SetDriveModifiers extends Command {

  private Dimensionless m_previousTranslationModifier;
  private Dimensionless m_previousRotationModifier;

  public SetDriveModifiers() {
    m_previousTranslationModifier = Robot.swerve.getTranslationModifier();
    m_previousRotationModifier = Robot.swerve.getRotationModifier();
  }

  @Override
  public void initialize() {
    Robot.swerve.setTranslationModifier(SWERVE.INTAKE_TRANSLATION_MODIFIER);
    Robot.swerve.setRotationModifier(SWERVE.INTAKE_ROTATION_MODIFIER);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.setTranslationModifier(m_previousTranslationModifier);
    Robot.swerve.setRotationModifier(m_previousRotationModifier);
  }
}
