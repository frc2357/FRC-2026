package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.floor.FloorStop;
import frc.robot.commands.intakerunner.IntakeRunnerStop;
import frc.robot.commands.kicker.KickerStop;
import frc.robot.commands.tunnel.TunnelStop;

//import frc.robot.commands.shooter.ShooterStop
//For when updating with Shooter subsystem.
public class StopAllMotors extends ParallelCommandGroup {

  public StopAllMotors() {
    super(
      new IntakeRunnerStop(),
      new FloorStop(),
      Robot.feeder.stopCommand(),
      new KickerStop(),
      new TunnelStop(),
      Robot.hood.stopCommand(),
      Robot.intakePivot.stopCommand(),
      Robot.shooter.stopCommand()
    );
  }
}
