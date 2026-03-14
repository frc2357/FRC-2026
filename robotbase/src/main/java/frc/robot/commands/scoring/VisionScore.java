package frc.robot.commands.scoring;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.drive.DrivePoseTargetingHub;
import frc.robot.commands.drive.SetActionDriveModifiers;
import java.util.function.Supplier;

public class VisionScore extends SequentialCommandGroup {

  public VisionScore(Supplier<Dimensionless> x, Supplier<Dimensionless> y) {
    super(
      new SetActionDriveModifiers(),
      new ParallelCommandGroup(
        new Score(
          () -> Robot.scoreCalculator.getCalculatedShooterVelocity(),
          () -> Robot.scoreCalculator.getCalculatedHoodAngle()
        ),
        new DrivePoseTargetingHub(x, y)
      )
    );
  }
}
