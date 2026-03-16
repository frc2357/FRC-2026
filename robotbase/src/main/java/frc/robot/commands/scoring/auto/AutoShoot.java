package frc.robot.commands.scoring.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.scoring.Score;

public class AutoShoot extends ParallelCommandGroup {

  public AutoShoot() {
    super(
      new Score(
        () -> Robot.scoreCalculator.getCalculatedShooterVelocity(),
        () -> Robot.scoreCalculator.getCalculatedHoodAngle()
      )
    );
  }
}
