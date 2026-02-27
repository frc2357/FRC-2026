package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.feeder.FeederStop;
import frc.robot.commands.floor.FloorStop;
import frc.robot.commands.intake.IntakeStop;

//import frc.robot.commands.shooter.ShooterStop
//For when updating with Shooter subsystem.
public class StopAllMotors extends ParallelCommandGroup {

  public StopAllMotors() {
    super(new IntakeStop(), new FloorStop(), new FeederStop());
  }
}
