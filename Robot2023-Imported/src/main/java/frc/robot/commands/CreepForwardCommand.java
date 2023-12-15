package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class CreepForwardCommand extends Command {
    private final DriveSubsystem m_drive;

    public CreepForwardCommand(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(0.1, 0, 0, false, true);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}