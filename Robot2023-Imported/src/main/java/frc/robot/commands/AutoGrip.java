package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoGrip extends Command {
    private final ClawSubsystem m_claw;
    private DigitalInput _Input =
    new DigitalInput(0);
    private boolean m_Finished = false;
    public AutoGrip(ClawSubsystem claw) {
        m_claw = claw;

        addRequirements(m_claw);
    }

    @Override 
    public void initialize(){
        m_Finished = false;
    }

    @Override
    public void execute() {
        if (inPosition()) {
            m_claw.grip();
            m_Finished = true;
        }
    }
    
    private boolean inPosition() {
        return !_Input.get(); 
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("AGFinished", m_Finished);
        return m_Finished;
    }
}
