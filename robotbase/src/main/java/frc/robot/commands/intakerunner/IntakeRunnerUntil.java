package frc.robot.commands.intakerunner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.function.BooleanSupplier;

public class IntakeRunnerUntil extends Command {

  private BooleanSupplier m_untilCondition;

  public IntakeRunnerUntil(BooleanSupplier untilCondition) {
    m_untilCondition = untilCondition;
    addRequirements(Robot.intake);
  }

  @Override
  public void initialize() {
    Robot.intake.setSpeed(Constants.INTAKE_RUNNER.TELEOP_INTAKING_SPEED);
  }

  @Override
  public boolean isFinished() {
    return m_untilCondition.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
