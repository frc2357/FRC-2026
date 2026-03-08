package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;

public abstract class AutoBase {

  protected final String m_name;

  public AutoBase(String name) {
    m_name = name;
  }

  public abstract AutoRoutine getRoutine();

  @Override
  public String toString() {
    return m_name;
  }
}
