// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RumbleCMD extends SequentialCommandGroup {
  /** Creates a new RumbleCMD. */
  public RumbleCMD(CommandXboxController driverController, CommandXboxController operatorController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    Commands.runOnce(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)),
    Commands.runOnce(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1)),
    Commands.waitSeconds(.5),
    Commands.runOnce(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)),
    Commands.runOnce(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }
}
