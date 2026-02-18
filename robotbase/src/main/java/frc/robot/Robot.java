// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Value;
import static frc.robot.Constants.CHOREO.AUTO_FACTORY;

import choreo.auto.AutoRoutine;
import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.StopAllMotors;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveSetCoast;
import frc.robot.commands.spindexer.SpindexerAxis;
import frc.robot.commands.util.InitRobotCommand;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.networkTables.AutoChooserManager;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.vision.CameraManager;
import frc.robot.vision.PhotonVisionCamera;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private static DriverControls m_driverControls;
  private static Command m_defaultDrive;

  public static CommandSwerveDrivetrain swerve;

  public static Spindexer spindexer;
  public static Alliance alliance = null;

  public static InitRobotCommand m_InitRobotCommand;

  private AutoChooserManager m_autoChooserManager;
  public static Intake intake;
  public static IntakePivot intakePivot;
  public static Shooter shooter;
  public static Hood hood;
  public static Outtake outtake;
  public static Feeder feeder;

  public static CameraManager cameraManager;
  public static ScoreCalculator scoreCalculator;

  private static final Field2d m_robotField = new Field2d();

  private final Telemetry logger = new Telemetry(
    Constants.SWERVE.MAX_SPEED.in(Units.MetersPerSecond)
  );
  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
    .withTimestampReplay()
    .withJoystickReplay();

  public Robot() {
    swerve = TunerConstants.createDrivetrain();

    intake = new Intake();
    intakePivot = new IntakePivot();
    shooter = new Shooter();
    hood = new Hood();
    spindexer = new Spindexer();
    outtake = new Outtake();
    feeder = new Feeder();

    cameraManager = new CameraManager();

    swerve.registerTelemetry(logger::telemeterize);
    scoreCalculator = new ScoreCalculator();

    m_driverControls = new DriverControls();
    m_defaultDrive = new DefaultDrive(
      m_driverControls::getLeftX,
      m_driverControls::getLeftY,
      m_driverControls::getRightX
    );

    swerve.registerTelemetry(logger::telemeterize);
    swerve.setDefaultCommand(m_defaultDrive);

    spindexer.setDefaultCommand(
      new SpindexerAxis(() -> {
        return Value.of(SmartDashboard.getNumber("Spindexer", 0.0));
      })
    );

    m_autoChooserManager = new AutoChooserManager();
    m_InitRobotCommand = new InitRobotCommand();

    SmartDashboard.putNumber("Spindexer", 0.0);

    // DON'T DELETE - Load the april tag field
    // This prevents a loop overrun when we first access the constants
    AprilTagFieldLayout layout = Constants.FieldConstants.FIELD_LAYOUT;

    SmartDashboard.putData("Robot Field", m_robotField);
  }

  @Override
  public void robotInit() {
    CommandScheduler.getInstance().schedule(m_InitRobotCommand);
  }

  @Override
  public void robotPeriodic() {
    Robot.cameraManager.updateResult();
    Robot.cameraManager.addSwerveEstimates(Robot.swerve::addVisionMeasurement);

    m_timeAndJoystickReplay.update();

    CommandScheduler.getInstance().run();

    m_robotField.setRobotPose(swerve.getFieldRelativePose2d());
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().schedule(new StopAllMotors());

    CommandScheduler.getInstance().schedule(
      new WaitCommand(SWERVE.TIME_TO_COAST).andThen(new DriveSetCoast())
    );
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    swerve.configNeutralMode(NeutralModeValue.Brake);

    m_autonomousCommand = m_autoChooserManager.getSelectedCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    swerve.configNeutralMode(NeutralModeValue.Brake);
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    swerve.configNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
