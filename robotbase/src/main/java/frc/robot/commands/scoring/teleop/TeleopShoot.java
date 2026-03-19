package frc.robot.commands.scoring.teleop;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.drive.DriveTargetLock;
import java.util.function.Supplier;

public class TeleopShoot extends ParallelCommandGroup {

  public TeleopShoot(Supplier<Dimensionless> x, Supplier<Dimensionless> y) {
    super(
      new TeleopScore(
        () -> Robot.shotCalculator.getCalculatedShooterVelocity(),
        () -> Robot.shotCalculator.getCalculatedHoodAngle()
      ),
      new DriveTargetLock(x, y)
    );
  }
}
