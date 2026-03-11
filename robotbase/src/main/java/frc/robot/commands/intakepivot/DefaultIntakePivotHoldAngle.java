package frc.robot.commands.intakepivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DefaultIntakePivotHoldAngle extends Command {

  public DefaultIntakePivotHoldAngle() {
    addRequirements(Robot.intakePivot);
  }

  @Override
  public void initialize() {
    Angle currentAngle = Robot.intakePivot.getAngle();

    Robot.intakePivot.setAngleSetpoint(currentAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intakePivot.stopMotor();
  }
}
