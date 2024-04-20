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

public class AlignToAMPCMD extends Command {
  private DriveTrain driveTrain;
  private CommandXboxController controller;
  private PIDController swerveRotationController;
    private PIDController swerveYController;
  private Vision visionSubsystem;
  /** Creates a new AlignToSpeakerCMD. */
  public AlignToAMPCMD(DriveTrain driveTrain, CommandXboxController controller, Vision visionSubsystem) {
    this.driveTrain = driveTrain;
    this.controller = controller;
    this.visionSubsystem = visionSubsystem;

    swerveRotationController = new PIDController(0.015, 0.0, 0.0005);
    swerveRotationController.setSetpoint(0);

    swerveYController = new PIDController(0.01, 0.0, 0.0);
    swerveYController.setSetpoint(0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ySpeed = swerveYController.calculate(visionSubsystem.getTranslationToAMP());
    double rotationalSpeed = swerveRotationController.calculate(visionSubsystem.getDegreesToAMP());
    driveTrain.drive(
          ySpeed,
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
