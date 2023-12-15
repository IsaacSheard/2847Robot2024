// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

// import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 5; // radians per second
        public static final double kMagnitudeSlewRate = 15; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 5; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(23);;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(23);;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kRearLeftDrivingCanId = 2;
        public static final int kFrontRightDrivingCanId = 3;
        public static final int kRearRightDrivingCanId = 4;

        public static final int kFrontLeftTurningCanId = 5;
        public static final int kRearLeftTurningCanId = 6;
        public static final int kFrontRightTurningCanId = 7;
        public static final int kRearRightTurningCanId = 8;

        public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants {
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kPinionTeeth = 14; // Adjust this to match your configuration!
        public static final double kMotorFreeSpeed = 5676 / 60;
        public static final double kDrivingMotorReduction = 990 / (kPinionTeeth * 15);
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDriveTrainFreeSpeed = (kMotorFreeSpeed * kWheelCircumferenceMeters)
                / kDrivingMotorReduction; // calculated motor free speed

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / (double) kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / (double) kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveTrainFreeSpeed;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1.2;
        public static final double kTurningI = 0;
        public static final double kTurningD = .02;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 40; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriverControllerPort1 = 1;
        public static final double kDriveDeadband = 0.10;
        public static final double kArmDeadband = 0.10;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 2;
        public static final double kPYController = 2;
        public static final double kPThetaController = 0;
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class FieldConstants {
        public static final long BluePickUpTagId = 4;
        public static final long RedPickUpTagId = 5;

        public static final long RedScore1 = 1;
        public static final long RedScore2 = 2;
        public static final long RedScore3 = 3;

        public static final long BlueScore6 = 6;
        public static final long BlueScore7 = 7;
        public static final long BlueScore8 = 8;
    }

    public static final class ArmConstants {
        public static final double kMaxEmptyGravityFF = 0.032;
        public static final double kMaxConeGravityFF = 0.038;
        public static final double kTimeout = 1.5;
        public static final double magnetOffset = -0.169921875;    //  0.2431640625 - 0.086669921875;
    }
}
