package frc.robot.triggers;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import yams.mechanisms.velocity.FlyWheel;

public class SetVelocity extends Command{
    public FlyWheel m_shooter;
    public AngularVelocity m_angularVelocity;

    public SetVelocity(AngularVelocity angularVelocity, FlyWheel shooter) {
        m_shooter = shooter;
        m_angularVelocity = angularVelocity;
    }

    @Override
    public void initialize() {
        
    }
}