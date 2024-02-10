// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClimbCommands.ClimbStart;
import frc.robot.commands.ClimbCommands.PigeonClimb.ClimbPigeonAwaitContact;
import frc.robot.commands.ClimbCommands.PigeonClimb.ClimbPigeonAwaitSecond;
import frc.robot.commands.ClimbCommands.PigeonClimb.ClimbPigeonRise;
import frc.robot.subsystems.Climb;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbPigeon extends SequentialCommandGroup {
  Climb climb;
  /** Creates a new ClimbPigeon. */
  public ClimbPigeon(Climb climb) {
    this.climb = climb;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClimbStart(climb),
      new ClimbPigeonAwaitContact(climb),
      new ClimbPigeonAwaitSecond(climb),
      new ClimbPigeonRise(climb)
    );
  }
}
