// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CMDStoTest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class RevAimAlignAutoFireCMD extends Command {
  private Shooter shooterSubsystem;
  private Vision visionSubsystem;
  private DriveTrain driveTrain;
  private PIDController swerveRotationController;
  /** Creates a new RevAimEndFireCMD. */
  public RevAimAlignAutoFireCMD(Shooter shooterSubsystem, Vision visionSubsystem, DriveTrain driveTrain) {
    this.shooterSubsystem = shooterSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.driveTrain = driveTrain;

    swerveRotationController = new PIDController(0.0065, 0.0, 0.0);
    swerveRotationController.setSetpoint(0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setFlywheelsRPM(ShooterConstants.shootingRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = visionSubsystem.getDistanceToSpeaker();
    if(distance != 0.0)
    {
      shooterSubsystem.setPivotAngle(shooterSubsystem.getAimingAngle(distance));
    }
    
    double rotationalSpeed = swerveRotationController.calculate(visionSubsystem.getDegreesToSpeaker());
    driveTrain.drive(
          0,
          0,
          rotationalSpeed,
          true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return visionSubsystem.facingSpeaker()&&shooterSubsystem.atDesiredAngle()&&shooterSubsystem.atDesiredRPM();
  }
}

