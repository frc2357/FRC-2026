package frc.robot.commands.passing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HOOD;
import frc.robot.Robot;

public class VisionTargetPassing extends Command {

  public VisionTargetPassing() {
    addRequirements(Robot.shooter, Robot.hood);
  }

  @Override
  public void initialize() {
    Robot.hood.setAngle(HOOD.PASSING_STATIC_ANGLE);
  }

  @Override
  public void execute() {
    Robot.shooter.setVelocitySetpoint(
      Robot.passCalculator.getCalculatedShooterVelocity()
    );
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stopMotor();
    Robot.hood.stopMotor();
  }
}
