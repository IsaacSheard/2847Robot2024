package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClawSubsystem;

public class ToggleClawCommand extends Command {

    private final ClawSubsystem m_claw;

    public ToggleClawCommand(ClawSubsystem claw) {
        m_claw = claw;

        addRequirements(m_claw);
    }

    @Override
    public void initialize() {
        if(m_claw.isClosed()){
            m_claw.release();
        } else {
            m_claw.grip();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
