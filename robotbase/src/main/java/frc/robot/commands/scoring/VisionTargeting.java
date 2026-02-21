package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ScoreCalculator.CalculatedShot;

public class VisionTargeting extends Command {

  public VisionTargeting() {
    addRequirements(Robot.shooter, Robot.hood);
  }

  @Override
  public void initialize() {
    Robot.cameraManager.setPipeline(Constants.PHOTON_VISION.MULTI_TAG_PIPELINE);
  }

  @Override
  public void execute() {
    CalculatedShot shot = Robot.scoreCalculator.calculateFromPose();

    //Robot.shooter.setVelocity(shot.shooterVelocity());
    //Robot.hood.setAngle(shot.hoodPosition());
  }

  public void end() {
    Robot.shooter.stopMotor();
    Robot.hood.stopMotor();
  }
}
