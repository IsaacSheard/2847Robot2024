package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmExtendSubsystem;

public class ToggleExtendReleaseCommand extends Command {

    private final ArmExtendSubsystem m_armExtend;

    public ToggleExtendReleaseCommand(ArmExtendSubsystem armExtend) {
        m_armExtend = armExtend;

        addRequirements(m_armExtend);
    }

    @Override
    public void initialize() {
        if(m_armExtend.isClosed()){
            m_armExtend.extend();
            System.out.println("extend");
        } else {
            m_armExtend.retract();
            System.out.println("retract");

        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
