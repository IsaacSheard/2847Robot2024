package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class StopArmCommand extends Command {
    private final ArmSubsystem m_arm;

    public StopArmCommand(ArmSubsystem arm) {
        m_arm = arm;

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
