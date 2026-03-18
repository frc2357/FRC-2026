package frc.robot.commands.scoring.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;

public class AutoShoot extends ParallelCommandGroup {

  public AutoShoot() {
    super(
      new AutoScore(
        () -> Robot.shotCalculator.getCalculatedShooterVelocity(),
        () -> Robot.shotCalculator.getCalculatedHoodAngle()
      )
    );
  }
}
