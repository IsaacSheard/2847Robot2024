// package frc.robot.subsystems;

// import com.playingwithfusion.TimeOfFlight;
// import com.playingwithfusion.TimeOfFlight.RangingMode;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class SensorSubsystem extends SubsystemBase {
//     public TimeOfFlight sensor = new TimeOfFlight(0);
//     private double m_range = 1500;

//     public SensorSubsystem() {
//         sensor.setRangingMode(RangingMode.Short, 24);
//     }
 
//      public boolean isRangeValid() {
//         return sensor.isRangeValid();
//      }
 
//      public double getRange() {
//         if (isRangeValid()) {
//             m_range = sensor.getRange();
//         } else{
//             m_range = 1500;
//         }
//         return m_range;
//      }
// }
