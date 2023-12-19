package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmExtendSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class ExtendWReleaseCommand extends Command {
    private final ArmExtendSubsystem m_armExtend;
    private final ClawSubsystem m_claw;
    private SequentialCommandGroup m_command;

    public ExtendWReleaseCommand(ArmExtendSubsystem arm, ClawSubsystem claw) {
        m_armExtend = arm;
        m_claw = claw;
        addRequirements(m_armExtend, m_claw);
    }

    @Override
    public void initialize() {
        m_command = new SequentialCommandGroup(
            new ToggleExtendReleaseCommand(m_armExtend),
            new WaitCommand(1),
            new ReleaseCommand(m_claw)
        );
        m_command.schedule();
    }

    @Override
    public boolean isFinished() {
        if (m_command == null) return false;
        return m_command.isFinished();
    }
}
