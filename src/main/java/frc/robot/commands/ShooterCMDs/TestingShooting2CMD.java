// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShooterCMDs.LowLevelCMDs.RunAMPFeedersCMD;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestingShooting2CMD extends SequentialCommandGroup {
  /** Creates a new TestingShooting2CMD. */
  public TestingShooting2CMD(Shooter shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(() -> shooterSubsystem.setFlywheelsShooting(1)),
      Commands.waitSeconds(3),
      new RunAMPFeedersCMD(ShooterConstants.feederShootingSpeed, shooterSubsystem),
      Commands.waitSeconds(1),
      Commands.runOnce(() -> shooterSubsystem.setFlywheelsShooting(0), shooterSubsystem),
      new RunAMPFeedersCMD(0, shooterSubsystem));
  }
}
