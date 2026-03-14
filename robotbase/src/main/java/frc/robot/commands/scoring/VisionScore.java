package frc.robot.commands.scoring;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.drive.DrivePoseTargetingHub;
import java.util.function.Supplier;

public class VisionScore extends ParallelCommandGroup {

  public VisionScore(Supplier<Dimensionless> x, Supplier<Dimensionless> y) {
    super(
      new Score(
        () -> Robot.shotCalculator.getCalculatedShooterVelocity(),
        () -> Robot.shotCalculator.getCalculatedHoodAngle()
      ),
      new DrivePoseTargetingHub(x, y)
    );
  }
}
