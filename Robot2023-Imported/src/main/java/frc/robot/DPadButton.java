// package frc.robot;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

// public class DPadButton extends Trigger {

//     XboxController xboxController;
//     Direction direction;

//     public DPadButton(XboxController xboxController, Direction direction) {
//         this.xboxController = xboxController;
//         this.direction = direction;
//     }

//     public static enum Direction {
//         UP(0), RIGHT(90), DOWN(180), LEFT(270);

//         int direction;

//         private Direction(int direction) {
//             this.direction = direction;
//         }
//     }

//     public boolean get() {
//         int dPadValue = xboxController.getPOV();
//         return (dPadValue == direction.direction) || (dPadValue == (direction.direction + 45) % 360)
//                 || (dPadValue == (direction.direction + 315) % 360);
//     }

// }