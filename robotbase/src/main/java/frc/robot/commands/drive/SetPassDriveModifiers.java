package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class SetPassDriveModifiers extends Command {

  public SetPassDriveModifiers() {}

  @Override
  public void initialize() {
    Robot.swerve.setTranslationModifier(SWERVE.PASS_TRANSLATION_MODIFIER);
    Robot.swerve.setRotationModifier(SWERVE.PASS_ROTATION_MODIFIER);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
