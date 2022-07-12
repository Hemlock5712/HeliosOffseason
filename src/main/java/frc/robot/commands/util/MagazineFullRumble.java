// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Magazine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MagazineFullRumble extends SequentialCommandGroup {
  /** Creates a new MagazineFullRumble. */
  public MagazineFullRumble(XboxController controller, Magazine magazine) {
    addCommands(
      // new RumbleCommand(controller, magazine).withTimeout(0.5),
      new WaitCommand(0.25)
      // new RumbleCommand(controller, magazine).withTimeout(0.5)
    );
  }
}
