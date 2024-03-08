// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightAnim;
import frc.robot.subsystems.Lights;

public class ringCMD extends Command {
  Lights CANdle;
  /** Creates a new ringCMD. */
  public ringCMD(Lights CANdle) {
    this.CANdle = CANdle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CANdle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CANdle.setAnimation(LightAnim.requestRing);
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
