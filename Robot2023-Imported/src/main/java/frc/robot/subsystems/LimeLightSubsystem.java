package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimeLightSubsystem extends SubsystemBase {
    public LimeLightSubsystem() {

    }

    public Pose3d fieldPosition() {

        var position = LimelightHelpers.getBotpose_wpiBlue("limelight");



        if (position.length == 7 || position.length == 6) {
            Translation3d tran3d = new Translation3d(position[0], position[1], (position[2]));
            Rotation3d r3d = new Rotation3d(position[3], position[4], position[5]);
            return new Pose3d(tran3d, r3d);
        }

        return null;
    }

    public Pose2d getVisionEstimatedPose() {

        double[] bot_pose;
  
        bot_pose = LimelightHelpers.getBotpose_wpiRed("limelight");
  
        if(bot_pose.length == 0) return new Pose2d();
  
        double bot_x = bot_pose[0];
        double bot_y = bot_pose[1];
        double rotation_z = (bot_pose[5] + 360) % 360;
  
  
        return new Pose2d(
          new Translation2d(bot_x, bot_y),
          Rotation2d.fromDegrees(rotation_z));
    }
  
    public double getLatency() {
        return Timer.getFPGATimestamp() - Units.millisecondsToSeconds(LimelightHelpers.getLatency_Pipeline("limelight"));
    }
  
    public long aprilTagId() {
      return ((long)LimelightHelpers.getFiducialID("limelight"));
    }
  
    public boolean isGoodTarget() {
      return LimelightHelpers.getTA("limelight") > 0.4;
  }

    // TODO: once Limelight is mounted on the bot, use the actual Limelight height
    // and angle
    public double distanceToAprilTagInches() {
        if (aprilTagId() <= 0) {
            return 0.0;
        } else {
            double targetOffSetAngle = LimelightHelpers.getTY("limelight");
            double limeLightAngle = 5;
            double limeLightHeight = 40;
            double targetHeight = 43;

            double angleToGoalDegrees = limeLightAngle + targetOffSetAngle;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);
            return (targetHeight - limeLightHeight) / Math.tan(angleToGoalRadians);
        }
    }

    // Array with aprilTag field positions for scoring (Tags 1,2,3 are for Red
    // Alliance, 6,7,8 are for Blue Alliance)
    // X and Y are location on the field, Yaw lets us know what direction we are
    // turned

    // Goal is to define field positions for scoring, then when we press and hold a
    // button for target positioning,
    // if a valid AprilTag is in view, we will read the botPose value from the
    // Limelight and take the X, Y and Yaw values.
    // Then we will use that position in relation to the desired scoring position
    // and turn the robot towards the desired
    // scoring position and drive forward at a certain speed. Once, reached, turn to
    // face the Target in order to score

    // TODO: define scoring positions based on testing
    private Pose3d[] scoringPositions = new Pose3d[] {
            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))
    };

    public double scorePositioningAngle() {
        long aprilTagId = aprilTagId();

        if (aprilTagId == -1)
            return -1;

        Pose3d currentBotPosition = fieldPosition();

        double scoreY = scoringPositions[(int) aprilTagId].getY();
        double scoreX = scoringPositions[(int) aprilTagId].getX();

        double currentY = currentBotPosition.getY();
        double currentX = currentBotPosition.getX();

        return java.lang.Math.atan((scoreY - currentY) / (scoreX - currentX));
    }
}
