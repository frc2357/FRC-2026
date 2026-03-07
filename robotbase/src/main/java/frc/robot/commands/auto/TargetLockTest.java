package frc.robot.commands.auto;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/*
 * This is a test auto to demonstrate the functionality of AutoBase (and so I can mentally verify the implementation makes sense)
 * None of the paths referenced here actually exist in Choreo
 */

public class TargetLockTest extends AutoBase {

  public TargetLockTest() {
    super("TargetLockTest");
    // Dash to center (no target lock) to intake fuel
    AutoTrajectory centerDash = setStartTrajectory(
      defaultTrajectory("centerDash")
    );
    // Return to alliance zone to begin shooting (this will end around the trench)
    AutoTrajectory returnToAllianceZone = defaultTrajectory(
      "returnToAllianceZone"
    );
    // Drive to outpost while target locking
    AutoTrajectory driveToOutpostTargetLock = targetLockTrajectory(
      "driveToOutpost"
    );

    centerDash.active().whileTrue(/* intake */ new InstantCommand());
    centerDash.done().onTrue(returnToAllianceZone.cmd());

    returnToAllianceZone
      .done()
      .onTrue(
        new ParallelCommandGroup(
          driveToOutpostTargetLock.cmd(),
          new ParallelDeadlineGroup(
            new WaitCommand(10),
            /* shoot */ new InstantCommand()
          )
        )
      );
  }
}
