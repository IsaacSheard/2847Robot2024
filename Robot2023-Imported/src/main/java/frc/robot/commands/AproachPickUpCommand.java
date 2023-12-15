// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.ClawSubsystem;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.SensorSubsystem;

// public class AproachPickUpCommand extends PIDCommand {
//     private final ClawSubsystem m_claw;
//     private final SensorSubsystem m_sensor;

//     private boolean m_Finished;

//     public AproachPickUpCommand(DriveSubsystem drive, ClawSubsystem claw, SensorSubsystem sensor) {
//         super(
//             new PIDController(1, 0, 0),
//             () -> sensor.getRange(),
//             210,
//             (output) -> {
//                 output /= -1000.0;
//                 SmartDashboard.putNumber("PID output", output);
//                 drive.drive(Math.min(output, 0.25), 0, drive.maintainHeading(0, 0.5, 0.40), true, true);
//             },
//             drive, sensor);
//         //getController().enableContinuousInput(0, 1500);
//         getController().setTolerance(3);
//         m_claw = claw;
//         m_sensor = sensor;
//         m_Finished = false;
//         addRequirements(claw);
//     }

//     @Override
//     public void initialize(){
//         m_Finished = false;
//     }

//     @Override
//     public void execute() {
//         super.execute();

//         SmartDashboard.putBoolean("AtSetpoint", getController().atSetpoint());
        
//         if (getController().atSetpoint()) {
//             m_claw.grip();
//             m_Finished = true;
//         } 

//         SmartDashboard.putNumber("range", m_sensor.getRange());
//         SmartDashboard.putBoolean("rangeValid", m_sensor.isRangeValid());        
//     }

//     @Override
//     public boolean isFinished() {
//         SmartDashboard.putBoolean("ApproachIsFinished", m_Finished);
//         return m_Finished;
//     }
// }
