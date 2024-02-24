// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Utils.Constants.OIConstants;
import frc.robot.commands.CollectAndIndexCMD;
import frc.robot.commands.DriveCMDs.DriveCMD;
import frc.robot.commands.DriveCMDs.SlowDriveCMD;
import frc.robot.commands.ShooterCMDs.TestShootingCMD;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public DriveTrain driveTrain;
  private Shooter shooterSubsystem;
  private Indexer indexerSubsystem;
  private Collector collectorSubsystem;

  // Controllers
  private CommandXboxController driverController = new CommandXboxController(OIConstants.driverControllerPort);
  private CommandXboxController operatorController = new CommandXboxController(OIConstants.operatorControllerPort);
  private final SendableChooser<Command> autoChooser;

  private SysIdRoutine sysIdRoutine;
  private Measure<Velocity<Voltage>> rampRate = Volts.of(.5).per(Second);
  private Measure<Voltage> stepVoltage = Volts.of(2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Subsystem Initialization
    driveTrain = new DriveTrain();
    shooterSubsystem = new Shooter();
    indexerSubsystem = new Indexer();
    collectorSubsystem = new Collector();

    // config default commands
    driveTrain.setDefaultCommand(new DriveCMD(driveTrain, driverController, true, false));

    // Config for Auto Chooser
    autoChooser = AutoBuilder.buildAutoChooser("NAME DEFAULT AUTO HERE");
    SmartDashboard.putData("Auto Chooser", autoChooser);


     sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(rampRate,stepVoltage, Second.of(10)),
      new SysIdRoutine.Mechanism(
    (voltage) -> shooterSubsystem.runVolts(voltage.in(Volts)),
    null, shooterSubsystem));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // drive controller configs
    driverController.rightBumper().whileTrue(new RunCommand(() -> driveTrain.setX(), driveTrain));
    driverController.leftBumper().whileTrue(new SlowDriveCMD(driveTrain, driverController, true, false));
    driverController.start().onTrue(new InstantCommand(() -> driveTrain.zeroHeading(), driveTrain));

    /*operatorController.povUp().and(shooterSubsystem::withinRange).whileTrue(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).andThen(()-> shooterSubsystem.stopVolts(), shooterSubsystem));
    operatorController.povRight().and(shooterSubsystem::withinRange).whileTrue(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).andThen(()-> shooterSubsystem.stopVolts(), shooterSubsystem));
    operatorController.povDown().and(shooterSubsystem::withinRange).whileTrue(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).andThen(()-> shooterSubsystem.stopVolts(), shooterSubsystem));
    operatorController.povLeft().and(shooterSubsystem::withinRange).whileTrue(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).andThen(()-> shooterSubsystem.stopVolts(), shooterSubsystem));
    operatorController.rightBumper().and(shooterSubsystem::withinRange).whileTrue(new RunVoltageCMD(2, shooterSubsystem));
    operatorController.leftBumper().and(shooterSubsystem::withinRange).whileTrue(new RunVoltageCMD(-2, shooterSubsystem));
    operatorController.start().onTrue(new InstantCommand(() -> shooterSubsystem.setEncoderPosition(90)));*/
    operatorController.a().whileTrue(new CollectAndIndexCMD(collectorSubsystem, indexerSubsystem, shooterSubsystem));
    operatorController.b().whileTrue(new TestShootingCMD(shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
