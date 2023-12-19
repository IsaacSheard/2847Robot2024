package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RotateLineupCommand extends Command {
    private final DriveSubsystem m_drive;
    private final double kYawTarget = 0; // degrees
    private final double kYawTolerance = 0.5; // degrees
    private final double kYawSpeed = 0.40;
    private final XboxController m_driverController1;

    public RotateLineupCommand(DriveSubsystem drive, XboxController driverController) {
        m_drive = drive;
        m_driverController1 = driverController;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(
            -MathUtil.applyDeadband(m_driverController1.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController1.getLeftX(), OIConstants.kDriveDeadband),
            m_drive.maintainHeading(kYawTarget, kYawTolerance, kYawSpeed),
            true,
            true);
    }

    @Override
    public boolean isFinished() {
        var finished = Math.abs(m_drive.yawError(kYawTarget)) <= kYawTolerance;
        SmartDashboard.putBoolean("ARFinished", finished);
        return finished;
    }

}
