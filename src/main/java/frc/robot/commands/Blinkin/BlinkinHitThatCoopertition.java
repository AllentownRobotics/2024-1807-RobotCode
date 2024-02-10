// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Blinkin;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.subsystems.Blinkin;

public class BlinkinHitThatCoopertition extends Command {
  Blinkin blinkin;
  
  /** Creates a new BlinkinCoopertition. */
  public BlinkinHitThatCoopertition(Blinkin blinkin) {
    this.blinkin = blinkin;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(blinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    blinkin.BlinkinSet(BlinkinConstants.blinkinGold);
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
