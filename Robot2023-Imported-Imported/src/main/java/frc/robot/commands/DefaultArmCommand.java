package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends Command {
    private final ArmSubsystem m_arm;
    private final XboxController m_xboxController;

    public DefaultArmCommand(ArmSubsystem arm, XboxController xboxController) {
        m_arm = arm;
        m_xboxController = xboxController;
        addRequirements(m_arm);

    }

    @Override
    public void initialize() {
        if (m_arm.setpoint() == null) {
            var position = m_arm.currentDegrees();
            var postion1 = m_arm.setpoint();
            if (position > -90) {
                setToPosition(position);
            }
        } else {
            var position = m_arm.currentDegrees();

            setToPosition(position);
        }
    }

    @Override
    public void execute() {
        var position = m_arm.setpoint();
        var stickValue = MathUtil.applyDeadband(m_xboxController.getLeftY(), OIConstants.kArmDeadband) * .7;
        if (position != null) {
            if(stickValue != 0){
                position = position + stickValue;
            }

            setToPosition(position);
        }
    }

    private void setToPosition(double position){
        if(m_arm.isSlow()){
            m_arm.setToPositionSlow(position);
        } else {
            m_arm.setToPosition(position);
        } 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
