package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class JumpBackCommand extends Command {
    private final DriveSubsystem m_drive;
    private final Timer m_timer = new Timer();
    public JumpBackCommand(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }
    @Override 
    public void initialize(){
        m_timer.reset();
        m_timer.start();

    }

    @Override
    public void execute() {
        m_drive.drive(-0.2, 0, 0, false, true);
        
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(1);
    }
  
}