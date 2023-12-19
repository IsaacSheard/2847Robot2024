package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtendSubsystem extends SubsystemBase {
    private final Solenoid m_solenoid;
    private final int Channel = 1;

    public ArmExtendSubsystem() {
        m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Channel);
    }

    public void extend() {
        m_solenoid.set(true);
    }

    public void retract() {
        m_solenoid.set(false);
    }
    public boolean isClosed() {
        return !m_solenoid.get();
    }
}
