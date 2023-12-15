// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.utils.SwerveUtils;

import edu.wpi.first.wpilibj2.command.*;

import com.ctre.phoenix6.hardware.Pigeon2;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor
    private final Pigeon2 m_gyro = new Pigeon2(11);

    //The Limelight
    private final LimeLightSubsystem m_limeLightSubsystem = new LimeLightSubsystem();

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
     */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0, 0, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0, 0, Units.degreesToRadians(30));

    private boolean positionInitilized = false;

    SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, 
        Rotation2d.fromDegrees(getAngle()), 
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }, 
        new Pose2d(), 
        stateStdDevs, 
        visionMeasurementStdDevs);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        // m_poseEstimator.update(
        //         Rotation2d.fromDegrees(-m_gyro.getAngle()),
        //         new SwerveModulePosition[] {
        //                 m_frontLeft.getPosition(),
        //                 m_frontRight.getPosition(),
        //                 m_rearLeft.getPosition(),
        //                 m_rearRight.getPosition()
        //         });


            if (!positionInitilized) {
                m_poseEstimator.resetPosition(Rotation2d.fromDegrees(getAngle()), new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
                }, new Pose2d(14.8, 4.6, Rotation2d.fromDegrees(getAngle())));

                positionInitilized = true;
            }

        // Add gyro reading to pose estimator
        m_poseEstimator.update(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        });

        double[] distanceMetersArray = new double[] {
            m_frontLeft.getPosition().distanceMeters, 
            m_frontRight.getPosition().distanceMeters, 
            m_rearLeft.getPosition().distanceMeters, 
            m_rearRight.getPosition().distanceMeters
        };

// if (!positionInitilized && m_limeLightSubsystem.isGoodTarget()){//need to look at this. Not working
//     m_poseEstimator.resetPosition( m_gyro.getRotation2d(),
//     new SwerveModulePosition[] {
//         m_frontLeft.getPosition(),
//         m_frontRight.getPosition(),
//         m_rearLeft.getPosition(),
//         m_rearRight.getPosition()
//     }, getPose());
//     positionInitilized = true;
// }

        SmartDashboard.putNumberArray("Encoder Distances (m)", distanceMetersArray);

        SmartDashboard.putBoolean("Is good target", m_limeLightSubsystem.isGoodTarget());
  
      // Also apply vision measurements if available.
      if (m_limeLightSubsystem.aprilTagId() != -1 && m_limeLightSubsystem.isGoodTarget()) {
        m_poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);

        Pose2d currentPosition = m_poseEstimator.getEstimatedPosition();//should we be using this? does the same this as getPose
        Pose2d visionPose = m_limeLightSubsystem.getVisionEstimatedPose();

    



            m_poseEstimator.addVisionMeasurement(
                visionPose,
                m_limeLightSubsystem.getLatency());
        } else {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(Double.MAX_VALUE,Double.MAX_VALUE,Double.MAX_VALUE));
        }

        var currentPose = getPose();//is this what continally updated botpose? Not 0,0 upon startup?
        SmartDashboard.putNumber("X", currentPose.getX());
        SmartDashboard.putNumber("Y", currentPose.getY());

        
          }


    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
                Rotation2d.fromDegrees(getAngle()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative

                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(-m_gyro.getAngle()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setStraight() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(180)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-180)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-180)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(180)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(getAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    private double getAngle() {
        return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    //     return new SequentialCommandGroup(
    //             new InstantCommand(() -> {
    //                 // Reset odometry for the first path you run during auto
    //                 if (isFirstPath) {
    //                     var initialState = PathPlannerTrajectory.transformStateForAlliance(
    //                             traj.getInitialState(), DriverStation.getAlliance());


    //                     this.resetOdometry(
    //                             new Pose2d(
    //                                     initialState.poseMeters.getTranslation(),
    //                                     initialState.holonomicRotation));
    //                 }
    //             }),
    //             new PPSwerveControllerCommand(
    //                     traj,
    //                     this::getPose,
    //                     DriveConstants.kDriveKinematics,
    //                     new PIDController(AutoConstants.kPXController, 0, 0), // X controller. Tune these values for
    //                                                                           // your robot. Leaving them 0 will only
    //                                                                           // use feedforwards.
    //                     new PIDController(AutoConstants.kPYController, 0, 0), // Y controller (usually the same values
    //                                                                           // as X controller)
    //                     new PIDController(AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these
    //                                                                               // values for your robot. Leaving them
    //                                                                               // 0 will only use feedforwards.
    //                     this::setModuleStates,
    //                     true,
    //                     this));

    // }

    public double getGyroRoll() {
        return m_gyro.getRoll().getValue();
    }

    public double getGyroPitch() {
        return m_gyro.getPitch().getValue();
    }

    public double getGyroYaw() {
        return m_gyro.getAngle();
    }

    public double maintainHeading(double yawTarget, double yawTolerance, double yawSpeed) {
        double rotSpeed = 0;
        var error = yawError(yawTarget);

        while (error > 180 || error < -180) {
            if (error > 0) error -= 360.0;
            else error += 360.0;
        }

        double slowYawSpeed = 0.020;

        if (error > yawTolerance) {
            rotSpeed = error > 15 ? yawSpeed : slowYawSpeed;
        } else if (error < -yawTolerance) {
            rotSpeed = -(error < -15 ? yawSpeed : slowYawSpeed);
        }

        SmartDashboard.putNumber("Heading Error", error);
        SmartDashboard.putNumber("Calculated Rotation speed", rotSpeed);

        return rotSpeed;
    }

    public double yawError(double yawTarget) {
        return ((getGyroYaw() - yawTarget) % 360);
    }

    // Configure the AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this // Reference to this subsystem to set requirements
    );
}

