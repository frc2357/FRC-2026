package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ScoreCalculator.CalculatedShot;

public class VisionTargeting extends Command {

  public VisionTargeting() {
    addRequirements(Robot.shooter, Robot.hood);
  }

  @Override
  public void execute() {
    CalculatedShot shot = Robot.scoreCalculator.calculateFromPose();

    Robot.shooter.setVelocitySetpoint(shot.shooterVelocity());
    //Robot.hood.setAngleSetpoint(shot.hoodPosition());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stopMotor();
    Robot.hood.stopMotor();
  }
}
