package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmExtendSubsystem;

public class ExtendCommand extends Command {
    private final ArmExtendSubsystem m_armExtend;

    public ExtendCommand(ArmExtendSubsystem armExtend) {
        m_armExtend = armExtend;

        addRequirements(m_armExtend);
    }
    
    @Override
    public void initialize() {
        m_armExtend.extend();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}