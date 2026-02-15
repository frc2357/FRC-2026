package frc.robot.commands.scoring;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.ScoreCalculator.CalculatedShot;

/**
 * This command is not representative of our actual scoring sequence
 * It was primarily created to validate the shooter curve
 */
public class Score extends Command {

  public Score() {
    SmartDashboard.putNumber("Fire Distance Inches", 0.0);
  }

  @Override
  public void execute() {
    Distance distance = Inches.of(
      SmartDashboard.getNumber("Fire Distance Inches", 0.0)
    );
    CalculatedShot shot = Robot.scoreCalculator.calculateShot(distance);
    Robot.shooter.setVelocitySetpoint(shot.shooterVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stopMotor();
  }
}
