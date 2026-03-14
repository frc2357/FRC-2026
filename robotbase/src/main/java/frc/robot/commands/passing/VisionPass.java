package frc.robot.commands.passing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;
import frc.robot.commands.drive.DrivePosePassing;
import frc.robot.commands.scoring.Pass;
import frc.robot.commands.scoring.SetShotTarget;
import java.util.function.Supplier;

public class VisionPass extends ParallelCommandGroup {

  public VisionPass(Supplier<Dimensionless> x, Supplier<Dimensionless> y) {
    super();
    addCommands(
      new Pass(
        () -> Robot.passCalculator.getCalculatedShooterVelocity(),
        () -> Constants.HOOD.PASSING_STATIC_ANGLE
      ),
      new DrivePosePassing(x, y).alongWith(new SetShotTarget(this::getTarget))
    );
  }

  private Translation2d getTarget() {
    // We are going to be targeting the center point of either bump (depending on what side of the field we're on)
    // Since our shooter curves are tuned to roughly intersect the point at a height of 72 inches,
    // the fuel should land past the bump inside of our alliance zone.
    Pose2d robotPose = Robot.swerve.getAllianceRelativePose2d();
    if (robotPose.getY() > FieldConstants.Hub.centerPoint.getY()) {
      return FieldConstants.Bump.Left.centerPoint;
    } else {
      return FieldConstants.Bump.Right.centerPoint;
    }
  }
}
