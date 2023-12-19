package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class ReleaseCommand extends Command {
    private final ClawSubsystem m_claw;

    public ReleaseCommand(ClawSubsystem claw) {
        m_claw = claw;

        addRequirements(m_claw);
    }

    @Override
    public void initialize() {
        m_claw.release();
        System.out.println("release");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
