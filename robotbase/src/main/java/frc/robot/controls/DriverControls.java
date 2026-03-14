package frc.robot.controls;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;
import frc.robot.commands.drive.DrivePoseTargetingHub;
import frc.robot.commands.drive.FlipPerspective;
import frc.robot.commands.drive.ResetPerspective;
import frc.robot.commands.intakepivot.IntakePivotDeploy;
import frc.robot.commands.intakepivot.IntakePivotJiggle;
import frc.robot.commands.intaking.TeleopIntake;
import frc.robot.commands.passing.VisionPass;
import frc.robot.commands.scoring.teleop.HubScore;
import frc.robot.commands.scoring.teleop.OutpostScore;
import frc.robot.commands.scoring.teleop.TeleopShoot;
import frc.robot.commands.scoring.teleop.TrenchScore;
import frc.robot.controls.util.RumbleInterface;

public class DriverControls implements RumbleInterface {

  private CommandXboxController m_controller;

  public DriverControls() {
    m_controller = new CommandXboxController(CONTROLLER.DRIVER_CONTROLLER_PORT);
    mapControls();
  }

  public void mapControls() {
    // Button chord for all buttons that cause intaking
    Trigger isIntaking = m_controller.leftTrigger();

    // Button chord for all buttons that cause shooting
    Trigger isShooting = m_controller
      .rightTrigger()
      .or(m_controller.rightBumper())
      .or(m_controller.leftBumper())
      .or(m_controller.y())
      .or(m_controller.x());

    m_controller.back().onTrue(new FlipPerspective());
    m_controller.start().onTrue(new ResetPerspective());

    m_controller.leftTrigger().whileTrue(new TeleopIntake());

    isShooting.and(isIntaking.negate()).whileTrue(new IntakePivotJiggle());
    isShooting.negate().onTrue(new IntakePivotDeploy());

    m_controller
      .rightTrigger()
      .whileTrue(new TeleopShoot(this::getLeftX, this::getLeftY));
    m_controller
      .leftBumper()
      .whileTrue(new VisionPass(this::getLeftX, this::getLeftY));
    // m_controller
    //   .leftBumper()
    //   .whileTrue(
    //     new Score(
    //       () ->
    //         RotationsPerSecond.of(
    //           SmartDashboard.getNumber("Shooter Target RPS", 0)
    //         ),
    //       () -> Degrees.of(SmartDashboard.getNumber("Hood Target Degree", 2))
    //     )
    //   );

    m_controller.rightBumper().whileTrue(new TrenchScore());
    m_controller.y().whileTrue(new OutpostScore());
    m_controller.x().whileTrue(new HubScore());

    m_controller
      .a()
      .whileTrue(new DrivePoseTargetingHub(this::getLeftX, this::getLeftY));

    m_controller
      .povLeft()
      .onTrue(
        new InstantCommand(() -> {
          var estimate =
            Robot.cameraManager.m_shooter.getSeedEstimateForSwerve();
          if (estimate.isPresent()) {
            Robot.swerve.setFieldRelativeTranslation2d(
              estimate.get().estimatedPose.toPose2d().getTranslation()
            );
          }
        })
      );
  }

  public Dimensionless getRightX() {
    return Value.of(modifyAxis(-m_controller.getRightX()));
  }

  public Dimensionless getLeftX() {
    return Value.of(modifyAxis(-m_controller.getLeftX()));
  }

  public Dimensionless getLeftY() {
    return Value.of(modifyAxis(-m_controller.getLeftY()));
  }

  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private double modifyAxis(double value) {
    value = deadband(value, CONTROLLER.DRIVER_CONTROLLER_DEADBAND);
    value = Math.copySign(
      Math.pow(value, Constants.CONTROLLER.JOYSTICK_RAMP_EXPONENT),
      value
    );
    return value;
  }

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
