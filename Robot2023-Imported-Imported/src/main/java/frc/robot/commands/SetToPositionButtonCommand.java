package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetToPositionButtonCommand extends Command {
    private final ArmSubsystem m_arm;
    private final double m_degrees;

    public SetToPositionButtonCommand(ArmSubsystem arm, double degrees) {
        m_degrees = degrees;
        m_arm = arm;

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setToPosition(m_degrees);
    }
    @Override
    public void execute() {
        m_arm.setToPosition(m_degrees);
    }

    
}
