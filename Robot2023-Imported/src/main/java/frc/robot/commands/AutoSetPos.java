package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AutoSetPos extends Command {
    private final ArmSubsystem m_arm;
    private final double m_degrees;
    private final double kTolerance = 5; // degrees
    private final double kTimeout = 5; // seconds
    private final Timer m_timer = new Timer();

    public AutoSetPos(ArmSubsystem arm, double degrees) {
        m_degrees = degrees;
        m_arm = arm;

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setToPosition(m_degrees);//change to slow
    }

    @Override
    public void execute() {
        m_arm.setToPosition(m_degrees);//change to slow
    }


    @Override
    public boolean isFinished() {
        return Math.abs(m_arm.currentPosition() - m_degrees) < kTolerance || m_timer.hasElapsed(kTimeout);

    }

}