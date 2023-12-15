package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmExtendSubsystem;

public class RetractCommand extends Command {
    private final ArmExtendSubsystem m_armExtend;

    public RetractCommand(ArmExtendSubsystem armExtend) {
        m_armExtend = armExtend;

        addRequirements(m_armExtend);
    }
    
    @Override
    public void initialize() {
        m_armExtend.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}