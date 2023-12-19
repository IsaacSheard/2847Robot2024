// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPoint;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.commands.AproachPickUpCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoGrip;
import frc.robot.commands.CreepForwardCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.ExtendCommand;
//import frc.robot.commands.GoToPointCommand;
import frc.robot.commands.GripCommand;
import frc.robot.commands.JumpBackCommand;
import frc.robot.commands.MaintainCommand;
import frc.robot.commands.MoveArmLowToDropConePositionCommand;
import frc.robot.commands.MoveArmHighToDropConePositionCommand;
import frc.robot.commands.MoveArmToHPCommand;
import frc.robot.commands.MoveArmToPickUpCommand;
import frc.robot.commands.ReleaseCommand;
import frc.robot.commands.RetractCommand;
import frc.robot.commands.RotateLineupCommand;
import frc.robot.commands.SetToPositionButtonCommand;
import frc.robot.commands.SetToPositionSlowButtonCommand;
import frc.robot.commands.StopArmCommand;
import frc.robot.commands.TeleopMoveArmHighToDropConePositionCommand;
import frc.robot.commands.ToggleClawCommand;
import frc.robot.commands.ToggleExtendReleaseCommand;

import frc.robot.subsystems.ArmExtendSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
// import frc.robot.subsystems.SensorSubsystem;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //private final SendableChooser<Command> autoChooser;

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final ArmExtendSubsystem m_armExtend = new ArmExtendSubsystem();
    private final ClawSubsystem m_claw = new ClawSubsystem();
    // private final SensorSubsystem m_sensor = new SensorSubsystem();
    private final SendableChooser<Integer> m_autoChooser = new SendableChooser<>();

    private final Pigeon2 m_gyro = new Pigeon2(11);
    private final  HashMap<String, Command> m_eventMap = new HashMap<>();
    private final LimeLightSubsystem m_limeLightSubsystem = new LimeLightSubsystem();

    private Command m_bottomCubeScoreOut;
    private Command m_excubeConeScoreParallel;
    private Command m_cubeScoreMiddle;



    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_driverController1 = new XboxController(OIConstants.kDriverControllerPort1);



    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Reset odometry on startup
        m_gyro.reset();

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController1.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController1.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController1.getRightX(), OIConstants.kDriveDeadband),
                                true, true),
                        m_robotDrive));

         m_arm.setDefaultCommand(new DefaultArmCommand(m_arm, m_driverController));  // makes it go to 0
       //  m_arm.setDefaultCommand(new DefaultArmCommand(m_arm, m_driverController));  // makes it go to 0

        m_autoChooser.addOption("CubeConeScoreParallel", 1);
        m_autoChooser.addOption("BottomCubeConeScoreParalel", 2);
        m_autoChooser.addOption("SCubeConeScoreParallel", 3);
        m_autoChooser.addOption("CubeScoreMiddle", 4);
        m_autoChooser.addOption("StationBottomCubeConeScoreParallel", 5);
        m_autoChooser.addOption("BottomCubeScoreOut", 6);
        m_autoChooser.addOption("CubeScoreCharge", 7);
        m_autoChooser.addOption("ExCubeConeScoreParallel", 8);


        SmartDashboard.putData("Autonomous Routine", m_autoChooser);


   NamedCommands.registerCommand("straight", new InstantCommand(
            () -> m_robotDrive.setStraight(),
            m_robotDrive));
        NamedCommands.registerCommand("allignWheels", allignWheels());
        NamedCommands.registerCommand("score", new MoveArmHighToDropConePositionCommand(m_arm));
        NamedCommands.registerCommand("extend", extendWithReleaseCommand(false));
        NamedCommands.registerCommand("retract", new ToggleExtendReleaseCommand(m_armExtend));
        NamedCommands.registerCommand("positionClaw", toPickupPositionAndReleaseForAuto());
        NamedCommands.registerCommand("pickup", new GripCommand(m_claw));
        NamedCommands.registerCommand("maintain", new MaintainCommand(m_arm));
        NamedCommands.registerCommand("score1", new MoveArmHighToDropConePositionCommand(m_arm));
        NamedCommands.registerCommand("extendWRelease", extendLowerWithReleaseCommand(true));
        NamedCommands.registerCommand("retract1", new ToggleExtendReleaseCommand(m_armExtend));
        NamedCommands.registerCommand("positionClaw1", new MoveArmToPickUpCommand(m_arm));
        NamedCommands.registerCommand("lock", new AutoBalance(m_robotDrive));
        NamedCommands.registerCommand("open", new ReleaseCommand(m_claw));
        NamedCommands.registerCommand("maintainPickup", maintainPickup());

        
//     m_eventMap.put("straight", new InstantCommand(
//             () -> m_robotDrive.setStraight(),
//             m_robotDrive));
//     m_eventMap.put("allignWheels", allignWheels());
//     m_eventMap.put("score", new MoveArmHighToDropConePositionCommand(m_arm));
//     m_eventMap.put("extend", extendWithReleaseCommand(false));
//     m_eventMap.put("retract", new ToggleExtendReleaseCommand(m_armExtend));
//     m_eventMap.put("positionClaw", toPickupPositionAndReleaseForAuto());
//     m_eventMap.put("pickup", new GripCommand(m_claw));
//     m_eventMap.put("maintain", new MaintainCommand(m_arm));                                   NOT SURE IF THIS WILL GET DELETED
//     m_eventMap.put("score1", new MoveArmHighToDropConePositionCommand(m_arm));
//     m_eventMap.put("extendWRelease", extendLowerWithReleaseCommand(true));
//     m_eventMap.put("retract1", new ToggleExtendReleaseCommand(m_armExtend));
//     m_eventMap.put("positionClaw1", new MoveArmToPickUpCommand(m_arm));
//     m_eventMap.put("lock", new AutoBalance(m_robotDrive));
//     m_eventMap.put("open", new ReleaseCommand(m_claw));
//     m_eventMap.put("maintainPickup", maintainPickup());


//     m_bottomCubeScoreOut = BottomCubeScoreOut();
//     m_excubeConeScoreParallel = ExCubeConeScoreParallel();
//     m_cubeScoreMiddle = CubeScoreMiddle();


    }

    public void robotPeriodic() {
        var position = m_limeLightSubsystem.fieldPosition();
        SmartDashboard.putNumber("arm angle", m_arm.currentPosition());
        SmartDashboard.putNumber("CurrentPosition_X", position.getX());
        SmartDashboard.putNumber("CurrentPosition_Y", position.getY());
         }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        // use with xbox controller

        new JoystickButton(m_driverController1, Button.kRightBumper.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

        new JoystickButton(m_driverController, Button.kRightBumper.value)
                .toggleOnTrue(new ToggleClawCommand(m_claw));

        new JoystickButton(m_driverController, Button.kBack.value)
                .whileTrue(new SetToPositionSlowButtonCommand(m_arm, 165));

        new JoystickButton(m_driverController, Button.kY.value)
                .whileTrue(new SetToPositionButtonCommand(m_arm, 165));

        new JoystickButton(m_driverController, Button.kStart.value)
                .whileTrue(new MaintainCommand(m_arm));
        new JoystickButton(m_driverController, Button.kX.value)
                .whileTrue(toPickupPositionAndRelease());

        new JoystickButton(m_driverController, Button.kA.value)
                .whileTrue(toScoreHigh());

        // new JoystickButton(m_driverController, Button.kY.value)
        //         .whileTrue(new RetractCommand(m_armExtend))
        //         .onFalse(new StopArmCommand(m_arm));

         new JoystickButton(m_driverController, Button.kB.value)
                .whileTrue(new ExtendCommand(m_armExtend))
                .onFalse(new StopArmCommand(m_arm));





        new JoystickButton(m_driverController, Button.kLeftBumper.value)
                .onTrue(toHumanPickupPositionAndRelease()); // new MoveArmToHPCommand(m_arm))
              //  .onFalse(new StopArmCommand(m_arm));



        // new DPadButton(m_driverController, DPadButton.Direction.UP)
        //         .whileTrue(new ToggleExtendReleaseCommand(m_armExtend));

         new JoystickButton(m_driverController1, Button.kY.value)
             .whileTrue(new RotateLineupCommand(m_robotDrive, m_driverController1));



        //     new JoystickButton(m_driverController1, Button.kA.value)
        //     .whileTrue(AllignSensors());

        //     new JoystickButton(m_driverController1, Button.kX.value)
        //     .whileTrue(AllignSensorsDist());

        // new JoystickButton(m_driverController1, Button.kB.value)
        //     //.onTrue(toScorePositionCommand(true));//nee the addrequirements so that m_drive can take control
        //     .whenPressed(() -> {
        //       CommandScheduler.getInstance().schedule(
        //           toScorePositionCommand(true));//look into how to do this with a public command
        //   }, m_robotDrive);

    }

//     public Command getAutonomousCommand() {
//         switch (m_autoChooser.getSelected()) {
//             case 1:
//                 return CubeConeScoreParallel();
//             case 2:
//                 return BottomCubeConeScoreParallel();
//             case 3:
//                 return SCubeConeScoreParallel();
//             case 4:
//                 return m_cubeScoreMiddle;
//             case 5:
//                 return StationBottomCubeConeScoreParallel();
//             case 6:
//                 return m_bottomCubeScoreOut;
//             case 7:
//                 return CubeScoreCharge();
//             case 8:
//                 return m_excubeConeScoreParallel;

//         }

//         return null;
   // }

    //private Command AutoBuilder(String pathName) {
        // This will load the file "FullAuto.path" and generate it with a max velocity
        // of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group

        // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(3, 2));


        // // Create the AutoBuilder. This only needs to be created once when robot code
        // // starts, not every time you want to create an auto command. A good place to
        // // put this is in RobotContainer along with your subsystems.
        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //         () -> m_robotDrive.getPose(),
        //         (p) -> m_robotDrive.resetOdometry(p), // Pose2d consumer, used to reset odometry at the beginning of
        //                                               // auto
        //         DriveConstants.kDriveKinematics, // SwerveDriveKinematics
        //         new PIDConstants(5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
        //                                        // and Y
        //                                        // PID controllers)
        //         new PIDConstants(7, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
        //                                        // rotation
        //                                        // controller)
        //         (s) -> m_robotDrive.setModuleStates(s), // Module states consumer used to output to the drive subsystem
        //         m_eventMap,
        //         true, // Should the path be automatically mirrored depending on alliance color.
        //               // Optional, defaults to true
        //         m_robotDrive // The drive subsystem. Used to properly set the requirements of path following
        //                      // commands
        // );

        // var command = autoBuilder.fullAuto(pathGroup);

        // return command;
  //  }






    // private Command CubeConeScoreParallel() {


    //     return AutoBuilder("CubeConeScoreParallel");
    // }

    // private Command BottomCubeConeScoreParallel() {


    //     return AutoBuilder("BottomCubeConeScoreParallel");
    // }

    // private Command StationBottomCubeConeScoreParallel() {

    //     return AutoBuilder("StationBottomCubeConeScoreParallel");
    // }

    // private Command SCubeConeScoreParallel() {

    //     return AutoBuilder("SCubeConeScoreParallel");
    // }

    // private Command CubeScoreMiddle() {

    //     return AutoBuilder("CubeScoreMiddle");
    // }

    // private Command BottomCubeScoreOut() {

    //     return AutoBuilder("BottomCubeScoreOut");

    // }

    // private Command CubeScoreCharge() {

    //     return AutoBuilder("CubeScoreCharge");

    // }
    // private Command ExCubeConeScoreParallel() {


    //     return AutoBuilder("ExCubeConeScoreParallel");
    // }



    private Command extendWithReleaseCommand(boolean forCone) {
        return new SequentialCommandGroup(
                new ToggleExtendReleaseCommand(m_armExtend),
                new WaitCommand(forCone ? 2.5 : .25),
                new ReleaseCommand(m_claw),
                new WaitCommand(0.5),
                new GripCommand(m_claw));

    }

    private Command extendLowerWithReleaseCommand(boolean forCone) {
        return new SequentialCommandGroup(
                new ToggleExtendReleaseCommand(m_armExtend),
                new WaitCommand(1),
                new MoveArmLowToDropConePositionCommand(m_arm),
                new ReleaseCommand(m_claw),
                new WaitCommand(0.5),
                new MoveArmHighToDropConePositionCommand(m_arm),
                new GripCommand(m_claw));
    }

    private Command toPickupPositionAndReleaseForAuto() {
        return new SequentialCommandGroup(
                new MoveArmToPickUpCommand(m_arm),
                new ReleaseCommand(m_claw));
    }

    private Command toPickupPositionAndRelease() {
        return new SequentialCommandGroup(
                new RetractCommand(m_armExtend),
                new ParallelCommandGroup(
                    new MoveArmToPickUpCommand(m_arm),
                    new SequentialCommandGroup(
                        new WaitCommand(1),
                        new GripCommand(m_claw))),
                new WaitCommand(1),
                new ReleaseCommand(m_claw));
    }

    private Command toHumanPickupPositionAndRelease() {
        return new SequentialCommandGroup(
               new GripCommand(m_claw),
                new MoveArmToHPCommand(m_arm),
                new ReleaseCommand(m_claw));
    }

    private Command toScoreHigh() {
        return new SequentialCommandGroup(
                new TeleopMoveArmHighToDropConePositionCommand(m_arm),
                new ExtendCommand(m_armExtend));
    }

    private Command maintainPickup() {
        return new SequentialCommandGroup(
            new GripCommand(m_claw),
            new WaitCommand(0.1),
            new MaintainCommand(m_arm));

    }
  


    private Command allignWheels(){
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> m_robotDrive.setStraight(),
                m_robotDrive),
            new InstantCommand(
                () -> m_gyro.reset())
        );
    }
  }

    // private Command AllignSensors() {

    //     return new SequentialCommandGroup(
    //         new RotateLineupCommand(m_robotDrive, m_driverController1),
    //         new ParallelRaceGroup(
    //             new AutoGrip(m_claw),
    //             new CreepForwardCommand(m_robotDrive)
    //         ),
    //         new WaitCommand(0.5),
    //         new JumpBackCommand(m_robotDrive)
    //     );

    // }

//     private Command AllignSensorsDist() {

//         return new SequentialCommandGroup(
//             new AproachPickUpCommand(m_robotDrive, m_claw, m_sensor),
//             new WaitCommand(0.5)
//         );

//     }

//     public Command toScorePositionCommand(boolean toRight) {//need to figure out how to do this in a public command
      
//         System.out.println("Testing toScorePositionCommand");
//         System.out.flush();

//         var from = currentPosition();
//         if (from == null) {
//             System.out.println("Cannot determine current position");
//             return new WaitCommand(0);
//         }

//         System.out.println("from is:" + from.toString());

//         var to = scorePosition(toRight);
//         if (to == null) {
//             System.out.println("Cannot determine target position");
//             return new WaitCommand(0);
//         }

//         System.out.println("to is:" + to.toString());

//         var pathToTest = PathPlanner.generatePath(new PathConstraints(4, 3), from, to);

       

//         return goToPointCommand(new PathConstraints(4, 3), from, to, false);

//       }

//       private Command toPickUpCommand(boolean toRight) {
//         var from = currentPosition();


//         if (from == null) {
//             System.out.println("Cannot determine current position");
//             return new WaitCommand(0);
//         }

//         var to = pickUpPosition(toRight);
//         if (to == null) {
//             System.out.println("Cannot determine target position");
//             return new WaitCommand(0);
//         }

//         return new WaitCommand(0);
//       }

//       private PathPoint currentPosition() {
//         var position = m_robotDrive.getPose();
//         if (position == null) return null;

//         SmartDashboard.putNumber("CurrentPosition_X", position.getX());
//         SmartDashboard.putNumber("CurrentPosition_Y", position.getY());


//         return new PathPoint(
//             position.getTranslation(),
//             position.getRotation());


//       }


//       private PathPoint scorePosition(boolean toRight) {
//         var tagId = m_limeLightSubsystem.aprilTagId();

//         // TODO: determine what these field positions should be
//         if (toRight) {
//           if (tagId == Constants.FieldConstants.RedScore1) {
//             return new PathPoint(new Translation2d(14.76, 0.38), new Rotation2d(3.142));
//           } else if (tagId == Constants.FieldConstants.RedScore2) {
//             return new PathPoint(new Translation2d(14.76, 2.18), new Rotation2d(3.142));
//           } else if (tagId == Constants.FieldConstants.RedScore3) {
//             return new PathPoint(new Translation2d(14.76, 3.88), new Rotation2d(3.142));

//           }else if (tagId == Constants.FieldConstants.BlueScore6) {
//             return new PathPoint(new Translation2d(1.72, 1.67), new Rotation2d(0));

//           }else if (tagId == Constants.FieldConstants.BlueScore7) {
//             return new PathPoint(new Translation2d(1.72, 3.29), new Rotation2d(0));

//           }else if (tagId == Constants.FieldConstants.BlueScore8){
//             return new PathPoint(new Translation2d(1.72, 5.09), new Rotation2d(0));

//           }
//         } else {
//           if (tagId == Constants.FieldConstants.RedScore1) {
//             return new PathPoint(new Translation2d(14.76, 1.60), new Rotation2d(3.142));
//           } else if (tagId == Constants.FieldConstants.RedScore2) {
//             return new PathPoint(new Translation2d(14.76, 3.32), new Rotation2d(3.142));

//           }  else if (tagId == Constants.FieldConstants.RedScore3) {
//             return new PathPoint(new Translation2d(14.76, 5.13), new Rotation2d(3.142));
//           }else if (tagId == Constants.FieldConstants.BlueScore6) {
//             return new PathPoint(new Translation2d(1.72, 3.89), new Rotation2d(0));

//           }else if (tagId == Constants.FieldConstants.BlueScore7) {
//             return new PathPoint(new Translation2d(1.72, 2.23), new Rotation2d(0));

//           }else if (tagId == Constants.FieldConstants.BlueScore8){
//             return new PathPoint(new Translation2d(1.72, 0.42), new Rotation2d(0));

//           }
//         }

//         return null;
//       }

//       private PathPoint pickUpPosition(boolean right) {
//         var tagId = m_limeLightSubsystem.aprilTagId();

//         if (tagId == FieldConstants.BluePickUpTagId) {
//             if (right) {
//               return new PathPoint(new Translation2d(15.91, 6.00), new Rotation2d(3.142));

//             } else {
//               return new PathPoint(new Translation2d(15.91, 7.34), new Rotation2d(3.142));

//             }
//         } else if (tagId == FieldConstants.RedPickUpTagId) {
//             if (right) {
//               return new PathPoint(new Translation2d(0.56, 7.47), new Rotation2d(0));

//             } else {
//               return new PathPoint(new Translation2d(0.56, 6.11), new Rotation2d(0));


//             }
//         }

//         return null;
//       }

//       private Command goToPointCommand(PathConstraints constraints, PathPoint from, PathPoint to, boolean isFirstPath) {
//         return m_robotDrive.followTrajectoryCommand(PathPlanner.generatePath(constraints, from, to), isFirstPath);//I need this explained to me
//       }
// }
