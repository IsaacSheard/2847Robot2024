package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClawSubsystem extends SubsystemBase {
    private final Compressor m_compressor;
    private final Solenoid m_solenoid;
    private final int Channel = 0;
    
    public ClawSubsystem() {
        m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Channel);

    }

    public boolean pressureSwitch() {
        return m_compressor.getPressureSwitchValue();
    }

    public void grip() {
        m_solenoid.set(false);
    }

    public void release() {
        m_solenoid.set(true);
    }

    public boolean isClosed() {
        return !m_solenoid.get();
    }
}
