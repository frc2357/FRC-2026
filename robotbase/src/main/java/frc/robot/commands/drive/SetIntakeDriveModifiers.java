package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class SetIntakeDriveModifiers extends Command {

  public SetIntakeDriveModifiers() {
    SmartDashboard.putNumber(
      "intake translation",
      SWERVE.INTAKE_TRANSLATION_MODIFIER.in(Value)
    );
    SmartDashboard.putNumber(
      "intake rotation",
      SWERVE.INTAKE_ROTATION_MODIFIER.in(Value)
    );
  }

  @Override
  public void initialize() {
    Robot.swerve.setTranslationModifier(
      Value.of(
        SmartDashboard.getNumber(
          "intake translation",
          SWERVE.INTAKE_TRANSLATION_MODIFIER.in(Value)
        )
      )
    );
    Robot.swerve.setRotationModifier(
      Value.of(
        SmartDashboard.getNumber(
          "intake rotation",
          SWERVE.INTAKE_ROTATION_MODIFIER.in(Value)
        )
      )
    );
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
