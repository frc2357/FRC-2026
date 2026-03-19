package frc.robot.commands.drive;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class SetScoreDriveModifiers extends Command {

  public SetScoreDriveModifiers() {}

  @Override
  public void initialize() {
    Robot.swerve.setTranslationModifier(SWERVE.SCORE_TRANSLATION_MODIFIER);
    Robot.swerve.setRotationModifier(SWERVE.SCORE_ROTATION_MODIFIER);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
