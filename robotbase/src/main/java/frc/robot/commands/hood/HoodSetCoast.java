package frc.robot.commands.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class HoodSetCoast extends Command {

  public HoodSetCoast() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    Robot.swerve.configNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
