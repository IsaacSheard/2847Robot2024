package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends Command {
    private final DriveSubsystem m_drive;
    private final double kPitchTarget = 0; // degrees
    private final double kPitchTolerance = 3; // degrees
    private final double kRollTarget = 0; // degrees
    private final double kRollTolerance = 3; // degrees
    private final double kSeekSpeed = 0.06;

    public AutoBalance(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        double xSpeed = 0;
        double ySpeed = 0;
        var seeking = false;
        var error = pitchError();
        SmartDashboard.putNumber("pitchError", error);
        if (error > kPitchTolerance) {
            ySpeed = -kSeekSpeed; // climb right
            seeking = true;
        } else if (error < -kPitchTolerance) {
            ySpeed = kSeekSpeed; // climb left
            seeking = true;
        }

        error = rollError();
        SmartDashboard.putNumber("rollError", error);

        if (error > kRollTolerance) {
            xSpeed = -kSeekSpeed; // climb backward
            seeking = true;
        } else if (error < -kRollTolerance) {
            xSpeed = kSeekSpeed; // climb forward
            seeking = true;
        }

        if (seeking)
            m_drive.drive(xSpeed, ySpeed, 0, false, true);
        else
            m_drive.setX();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double rollError() {
        return m_drive.getGyroRoll() - kRollTarget;
    }

    private boolean rollInTolerance() {
        return Math.abs(rollError()) <= kRollTolerance;
    }

    private double pitchError() {
        return m_drive.getGyroPitch() - kPitchTarget;
    }

    private boolean pitchInTolerance() {
        return Math.abs(pitchError()) <= kPitchTolerance;
    }
}