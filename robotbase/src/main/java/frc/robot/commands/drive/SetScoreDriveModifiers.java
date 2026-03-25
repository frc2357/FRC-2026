package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class SetScoreDriveModifiers extends Command {

  public SetScoreDriveModifiers() {
    SmartDashboard.putNumber(
      "score translation",
      SWERVE.SCORE_TRANSLATION_MODIFIER.in(Value)
    );
    SmartDashboard.putNumber(
      "score rotation",
      SWERVE.SCORE_ROTATION_MODIFIER.in(Value)
    );
  }

  @Override
  public void initialize() {
    Robot.swerve.setTranslationModifier(
      Value.of(
        SmartDashboard.getNumber(
          "score translation",
          SWERVE.SCORE_TRANSLATION_MODIFIER.in(Value)
        )
      )
    );
    Robot.swerve.setRotationModifier(
      Value.of(
        SmartDashboard.getNumber(
          "score rotation",
          SWERVE.SCORE_ROTATION_MODIFIER.in(Value)
        )
      )
    );
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
