// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.util.ArmsIn;
import frc.robot.commands.util.TurretAlignToBack;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

public class ClimberArmsIn extends SequentialCommandGroup {

    public ClimberArmsIn(Climber climber, Turret turret) {
        addCommands(
                new TurretAlignToBack(turret),
                new ArmsIn(climber));
    }
}
