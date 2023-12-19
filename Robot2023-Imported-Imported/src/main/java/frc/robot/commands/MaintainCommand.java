package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class MaintainCommand extends Command {
    private final ArmSubsystem m_arm;    
    private final Timer m_timer = new Timer();
    private final double kAngle = -41; // degrees
    private final double kTolerance = 3; // degrees

    public MaintainCommand(ArmSubsystem arm) {
        m_arm = arm;

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setToPositionSlow(kAngle);
        m_timer.reset();
        m_timer.start();
    }
  

    @Override
    public void execute() {
        m_arm.setToPositionSlow(kAngle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_arm.currentPosition() - kAngle) < kTolerance || m_timer.hasElapsed(ArmConstants.kTimeout);    }
    }


