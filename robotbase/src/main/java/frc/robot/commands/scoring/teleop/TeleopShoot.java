package frc.robot.commands.scoring.teleop;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.drive.DrivePoseTargetingHub;
import frc.robot.commands.drive.SetActionDriveModifiers;
import frc.robot.commands.scoring.Score;
import java.util.function.Supplier;

public class TeleopShoot extends ParallelCommandGroup {

  public TeleopShoot(Supplier<Dimensionless> x, Supplier<Dimensionless> y) {
    super(
      new SetActionDriveModifiers(),
      new Score(
        () -> Robot.scoreCalculator.getCalculatedShooterVelocity(),
        () -> Robot.scoreCalculator.getCalculatedHoodAngle()
      ),
      new DrivePoseTargetingHub(x, y)
    );
  }
}
