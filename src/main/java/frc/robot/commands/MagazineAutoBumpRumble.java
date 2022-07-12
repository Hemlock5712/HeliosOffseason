// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.util.RumbleCommand;
import frc.robot.commands.wait.WaitForFullMagazine;
import frc.robot.commands.wait.WaitForNotFullMagazine;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Rumble;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MagazineAutoBumpRumble extends SequentialCommandGroup {
  /** Creates a new MagazineAutoBumpRumble. */
  public MagazineAutoBumpRumble(XboxController controller, Rumble rumble, Magazine magazine) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(rumble);
    addCommands(
      new WaitForFullMagazine(magazine),
      new RumbleCommand(controller, rumble),
      new WaitForNotFullMagazine(magazine)
    );
  }
}
