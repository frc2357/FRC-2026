package frc.robot.commands.pit;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.ResetPerspective;
import frc.robot.commands.drive.VelocityDrive;

public class TestSwerve extends SequentialCommandGroup {

  public TestSwerve() {
    super(
      new InstantCommand(() -> new ResetPerspective()),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new VelocityDrive(
          MetersPerSecond.of(1),
          MetersPerSecond.zero(),
          RadiansPerSecond.zero()
        )
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new VelocityDrive(
          MetersPerSecond.zero(),
          MetersPerSecond.of(1),
          RadiansPerSecond.zero()
        )
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new VelocityDrive(
          MetersPerSecond.zero(),
          MetersPerSecond.zero(),
          RadiansPerSecond.of(2)
        )
      )
    );
  }
}
