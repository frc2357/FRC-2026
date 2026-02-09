package frc.robot.commands.scoring;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.feeder.FeederAxis;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.spindexer.SpindexerAxis;
import frc.robot.commands.spindexer.SpindexerSetSpeed;

public class FeedAndSpin extends SequentialCommandGroup {

  public FeedAndSpin() {
    super(
      new ParallelCommandGroup(
        new SpindexerAxis(() ->
          Percent.of(SmartDashboard.getNumber("Spindexer Speed", 30))
        ),
        new WaitCommand(1).andThen(
          new FeederAxis(() ->
            Percent.of(SmartDashboard.getNumber("Feeder Speed", 30))
          )
        )
      )
    );
  }
}
