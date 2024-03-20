// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AlignToSpeakerCMD extends Command {
  private DriveTrain driveTrain;
  private CommandXboxController controller;
  private PIDController swerveRotationController;
  private Vision visionSubsystem;
  /** Creates a new AlignToSpeakerCMD. */
  public AlignToSpeakerCMD(DriveTrain driveTrain, CommandXboxController controller, Vision visionSubsystem) {
    this.driveTrain = driveTrain;
    this.controller = controller;
    this.visionSubsystem = visionSubsystem;

    swerveRotationController = new PIDController(0.0075, 0.0, 0.0);
    swerveRotationController.setSetpoint(0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationalSpeed = swerveRotationController.calculate(visionSubsystem.getDegreesToSpeaker());
    driveTrain.drive(
          MathUtil.applyDeadband(controller.getLeftY(), 0.1),
          MathUtil.applyDeadband(controller.getLeftX(), 0.1),
          rotationalSpeed,
          true, false);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
