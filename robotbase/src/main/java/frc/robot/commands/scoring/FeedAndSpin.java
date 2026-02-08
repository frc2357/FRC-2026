package frc.robot.commands.scoring;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.spindexer.SpindexerSetSpeed;

public class FeedAndSpin extends SequentialCommandGroup {

  public FeedAndSpin() {
    super(
      new InstantCommand(() -> {
        SmartDashboard.putNumber("Spindexer Speed", 30);
        SmartDashboard.putNumber("Feeder Speed", 30);
      }),
      new ParallelCommandGroup(
        new SpindexerSetSpeed(
          Percent.of(SmartDashboard.getNumber("Spindexer Speed", 30))
        ),
        new FeederSetSpeed(
          Percent.of(SmartDashboard.getNumber("Feeder Speed", 30))
        )
      )
    );
  }
}
