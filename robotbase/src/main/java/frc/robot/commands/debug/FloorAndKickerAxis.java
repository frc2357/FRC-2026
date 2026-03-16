package frc.robot.commands.debug;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.floor.FloorAxis;
import frc.robot.commands.kicker.KickerAxis;
import java.util.function.Supplier;

public class FloorAndKickerAxis extends ParallelCommandGroup {

  public FloorAndKickerAxis(Supplier<Dimensionless> axis) {
    super(new FloorAxis(axis), new KickerAxis(axis));
  }
}
