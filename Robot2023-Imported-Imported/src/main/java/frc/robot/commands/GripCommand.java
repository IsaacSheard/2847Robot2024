package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClawSubsystem;

public class GripCommand extends InstantCommand {
    public GripCommand(ClawSubsystem claw) {
        super(() -> claw.grip());
    }
}
