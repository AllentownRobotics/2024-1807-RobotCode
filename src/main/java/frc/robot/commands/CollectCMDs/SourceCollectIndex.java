// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class SourceCollectIndex extends Command {
  Shooter shootersubsystem;
  /** Creates a new SourceCollectIndex. */
  public SourceCollectIndex(Shooter shootersubsystem) {
    this.shootersubsystem = shootersubsystem;
    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(shootersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootersubsystem.runFlywheelsShooting(ShooterConstants.sourceCollectionSpeed);
    shootersubsystem.setAMPFeeder(ShooterConstants.feederSourceSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootersubsystem.runFlywheelsShooting(0);
    shootersubsystem.setAMPFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shootersubsystem.getBeamBreakSource();
  }
}
