package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class SetIntakeDriveModifiers extends Command {

  public SetIntakeDriveModifiers() {}

  @Override
  public void initialize() {
    Robot.swerve.setTranslationModifier(SWERVE.INTAKE_TRANSLATION_MODIFIER);
    Robot.swerve.setRotationModifier(SWERVE.INTAKE_ROTATION_MODIFIER);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
