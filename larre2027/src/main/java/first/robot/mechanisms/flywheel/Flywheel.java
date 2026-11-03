package first.robot.mechanisms.flywheel;

import static org.wpilib.units.Units.Value;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Dimensionless;
import first.robot.constants.CanID;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.StrictFollower;

public class Flywheel implements Mechanism {
    
    public static final TalonFX m_leftMotor = new TalonFX(CanID.LEFT_SHOOTER_MOTOR, new CANBus(CANPort.CAN_S1));
    public static final TalonFX m_rightMotor = new TalonFX(CanID.RIGHT_SHOOTER_MOTOR, new CANBus(CANPort.CAN_S1));
   
public Flywheel() {
          m_rightMotor.setControl(new StrictFollower(m_leftMotor.getDeviceID()));
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_leftMotor.getConfigurator().apply(leftConfig);
        m_rightMotor.getConfigurator().apply(rightConfig);
    }

    private void setSpeed(Dimensionless m_speed) {
        m_leftMotor.setThrottle(m_speed.in(Value));
    }

    private void setAxisSpeed(Dimensionless axisSpeed) {
        setSpeed(axisSpeed.times(FlywheelConstants.AXIS_MAX_SPEED));
    }
    public void stop() {
        m_leftMotor.stopMotor();
    }

    public Command FlywheelSetSpeed(Dimensionless speed) {
        return this.run((coro) -> {
            setSpeed(speed);
        }).named("Set Flywheel Speed");
        }

        public Command FlywheelAxis(Supplier<Dimensionless> axis) {
    return this.run((coro) -> {
         setSpeed(axis.get());
    }).named("Set Flywheel Axis");
    }

    public Command FlywheelStop() {
    return this.run((coro)-> {
        stop();
    }).named("Flywheel stop");
}}