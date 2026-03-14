package frc.robot.commands.passing;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.drive.DrivePosePassing;
import frc.robot.commands.scoring.Score;
import java.util.function.Supplier;

public class VisionPass extends ParallelCommandGroup {

  public VisionPass(Supplier<Dimensionless> x, Supplier<Dimensionless> y) {
    super();
    addCommands(
      new Score(
        () -> Robot.shotCalculator.getCalculatedShooterVelocity(),
        () -> Constants.HOOD.PASSING_STATIC_ANGLE
      ),
      new DrivePosePassing(x, y)
    );
  }
}
