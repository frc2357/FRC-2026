package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;
import frc.robot.commands.drive.CrossWheels;
import frc.robot.commands.intakepivot.IntakePivotPit;
import frc.robot.commands.intakerunner.IntakeRunnerSetSpeed;
import frc.robot.commands.pit.CleanFeed;
import frc.robot.commands.pit.SlowPitShoot;
import frc.robot.commands.pit.TestSwerve;

public class PitControls {

  private CommandXboxController m_controller;

  public PitControls() {
    m_controller = new CommandXboxController(CONTROLLER.PIT_CONTROLLER_PORT);
    mapControls();
  }

  public void mapControls() {
    m_controller.b().whileTrue(new SlowPitShoot());

    m_controller.x().onTrue(new CrossWheels());

    m_controller.a().whileTrue(new TestSwerve());

    m_controller.y().whileTrue(new IntakePivotPit());

    m_controller
      .rightBumper()
      .whileTrue(
        new IntakeRunnerSetSpeed(Constants.INTAKE_RUNNER.TELEOP_INTAKING_SPEED)
      );

    m_controller.leftBumper().whileTrue(new CleanFeed());

    m_controller
      .povUp()
      .whileTrue(Robot.hood.setSpeed(Constants.HOOD.MANUAL_HOOD_SPEED));

    m_controller
      .povDown()
      .whileTrue(
        Robot.hood.setSpeed(Constants.HOOD.MANUAL_HOOD_SPEED.times(-1))
      );
  }
}
