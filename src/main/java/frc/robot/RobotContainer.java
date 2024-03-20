// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ClimbCMD;
import frc.robot.commands.PanicButtonCMD;
import frc.robot.commands.ShootInAutoCMD;
import frc.robot.commands.CMDStoTest.RevAimEndFireCMD;
import frc.robot.commands.CollectCMDs.GroundCollectIndexCMD;
import frc.robot.commands.CollectCMDs.GroundCollectUntilBreakCMD;
import frc.robot.commands.CollectCMDs.SourceCollectIndex;
import frc.robot.commands.DriveCMDs.AlignToSpeakerCMD;
import frc.robot.commands.DriveCMDs.DriveCMD;
import frc.robot.commands.DriveCMDs.RotateToSpeakerCMD;
import frc.robot.commands.DriveCMDs.SlowDriveCMD;
import frc.robot.commands.ShooterCMDs.CollectSourceCMD;
import frc.robot.commands.ShooterCMDs.FeedShotFromMidLine;
import frc.robot.commands.ShooterCMDs.FeedShotFromWingLine;
import frc.robot.commands.ShooterCMDs.HalfResetShooterCMD;
import frc.robot.commands.ShooterCMDs.ResetShooterCMD;
import frc.robot.commands.ShooterCMDs.ScoreAMPCMD;
import frc.robot.commands.ShooterCMDs.SelfPodiumShotCMD;
import frc.robot.commands.ShooterCMDs.SelfShootAnyStraightCMD;
import frc.robot.commands.ShooterCMDs.SelfSubShotCMD;
import frc.robot.commands.ShooterCMDs.LowLevelCMDs.ManShootCurrentAngleCMD;
import frc.robot.commands.ShooterCMDs.LowLevelCMDs.SetPivotAngleCMD;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


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
  private Climb climbSubsystem;
  private Vision visionSubsystem;

  // Controllers
  private CommandXboxController driverController = new CommandXboxController(OIConstants.driverControllerPort);
  private CommandXboxController operatorController = new CommandXboxController(OIConstants.operatorControllerPort);
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Subsystem Initialization
    driveTrain = new DriveTrain();
    shooterSubsystem = new Shooter();
    indexerSubsystem = new Indexer();
    collectorSubsystem = new Collector();
    climbSubsystem = new Climb();
    visionSubsystem = new Vision();

    // config default commands
    driveTrain.setDefaultCommand(new DriveCMD(driveTrain, driverController, true, false));
    climbSubsystem.setDefaultCommand(new ClimbCMD(operatorController, climbSubsystem));
    //climbSubsystem.setDefaultCommand(new ZeroClimbCMD(climbSubsystem));

    //config Named Commands
    NamedCommands.registerCommand("ResetShooter", new HalfResetShooterCMD(shooterSubsystem));
    NamedCommands.registerCommand("Collect", new GroundCollectIndexCMD(collectorSubsystem, indexerSubsystem, shooterSubsystem, visionSubsystem));
    NamedCommands.registerCommand("SelfSubShot", new SelfSubShotCMD(shooterSubsystem, driverController, driveTrain, visionSubsystem));
    NamedCommands.registerCommand("SelfShootAnyStraight", new SelfShootAnyStraightCMD(shooterSubsystem, driverController, driveTrain, visionSubsystem));
    NamedCommands.registerCommand("Align", new RotateToSpeakerCMD(driveTrain, visionSubsystem));
    NamedCommands.registerCommand("WaitShooter", Commands.waitSeconds(ShooterConstants.shooterAutoWait));
    NamedCommands.registerCommand("ShootInAuto", new ShootInAutoCMD(shooterSubsystem, visionSubsystem, driveTrain));
    NamedCommands.registerCommand("Collect Pt1", new GroundCollectUntilBreakCMD(collectorSubsystem));

    // Config for Auto Chooser
    autoChooser = AutoBuilder.buildAutoChooser("NAME DEFAULT AUTO HERE");
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
    driverController.leftTrigger().whileTrue(new AlignToSpeakerCMD(driveTrain, driverController, visionSubsystem));
    driverController.rightTrigger().onTrue(new RotateToSpeakerCMD(driveTrain, visionSubsystem));
    driverController.a().whileTrue(driveTrain.generateAndFollowPath("PathFindToBlueAMP"));
    driverController.b().onTrue(new InstantCommand(() -> driveTrain.setHeadingToPoseHeading()));

    //operator controller configs
    //planned controls delete line when implemented
    /*Left Trigger - ManShotAnyStraight
     * Right Trigger - SelfShotAnyStraight
     * 
     * Left Bumper - Source Collect
     * Right Bumper - AMPScore
     * 
     * Left Stick - Climb
     * Right Stick - 
     * 
     * Y - Reset Shooter
     * A - Collect
     * 
     * povUp - SelfSubShot
     * povDown - SelfPodiumShot
     */
    operatorController.leftTrigger().whileTrue(new RevAimEndFireCMD(shooterSubsystem, visionSubsystem));
    operatorController.rightTrigger().whileTrue(new RevAimEndFireCMD(shooterSubsystem, visionSubsystem));
    operatorController.leftBumper().whileTrue(new CollectSourceCMD(shooterSubsystem)).onFalse(new ResetShooterCMD(shooterSubsystem));
    operatorController.rightBumper().onTrue(new SetPivotAngleCMD(ShooterConstants.preAMPAngle, shooterSubsystem)).onFalse(new ScoreAMPCMD(shooterSubsystem));
    operatorController.y().onTrue(new ResetShooterCMD(shooterSubsystem));
    operatorController.a().whileTrue(new GroundCollectIndexCMD(collectorSubsystem, indexerSubsystem, shooterSubsystem, visionSubsystem));
    operatorController.povDown().onTrue(new SelfSubShotCMD(shooterSubsystem, driverController, driveTrain, visionSubsystem));
    operatorController.povRight().whileTrue(new FeedShotFromMidLine(shooterSubsystem));
    operatorController.povUp().whileTrue(new FeedShotFromWingLine(shooterSubsystem));
    operatorController.start().whileTrue(new PanicButtonCMD(collectorSubsystem, indexerSubsystem, shooterSubsystem));

    operatorController.back().onTrue(new SetPivotAngleCMD(90, shooterSubsystem));
    operatorController.b().onTrue(Commands.runOnce(() -> shooterSubsystem.incrementSetpoit(1), shooterSubsystem));
    operatorController.x().onTrue(Commands.runOnce(() -> shooterSubsystem.incrementSetpoit(-1), shooterSubsystem));
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
