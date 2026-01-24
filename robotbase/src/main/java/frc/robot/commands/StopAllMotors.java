package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.spindexer.SpindexerStop;

//import frc.robot.commands.shooter.ShooterStop
//For when updating with Shooter subsystem.
public class StopAllMotors extends ParallelCommandGroup {

  public StopAllMotors() {
    super(new IntakeStop(), new SpindexerStop());
  }
}
