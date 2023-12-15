package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final int MotorCanId = 12;
    private final TalonFX m_motor = new TalonFX(MotorCanId);
    private final CANcoder m_CanCoder = new CANcoder(13);
    
    private Double m_positionSetpoint = null;
    private double m_degrees = 0;
    private boolean m_slow = false;
    private double kMMAccel = 600;
    private double kMMVel = 300;
    private double maxVelocity = 0;

    StatusSignal<Boolean> f_fusedSensorOutOfSinc = m_motor.getFault_FusedSensorOutOfSync();
    StatusSignal<Boolean> sf_fusedSensorOutOfSinc = m_motor.getStickyFault_FusedSensorOutOfSync();
   
    StatusSignal<Double> fx_pos = m_motor.getPosition();
    StatusSignal<Double> fx_vel = m_motor.getVelocity();
    StatusSignal<Double> cc_pos = m_CanCoder.getPosition();
    StatusSignal<Double> cc_vel = m_CanCoder.getVelocity();

    VelocityVoltage m_velocity = new VelocityVoltage(0);
    MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

    int printCount = 0;

    private final double kHorizontalPosition = -0.31640625;

    public ArmSubsystem() {

     // private SlewRateLimiter m_magLimiter = new SlewRateLimiter(ArmConstants.);

        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;

      //  slot0Configs.kS = 0.24;
       // slot0Configs.kV = 0.12;


        slot0Configs.kP = 100; //1.4;  
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        
       
        
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 2; //300   /80
        motionMagicConfigs.MotionMagicAcceleration = 4.5;  //600   //160
        motionMagicConfigs.MotionMagicJerk = 10;
     

        var cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset =  ArmConstants.magnetOffset; 
        m_CanCoder.getConfigurator().apply(cc_cfg);

        talonFXConfigs.Feedback.FeedbackRemoteSensorID = m_CanCoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfigs.Feedback.SensorToMechanismRatio = 1; 
        talonFXConfigs.Feedback.RotorToSensorRatio = 106.25;

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; 


        m_motor.getConfigurator().apply(talonFXConfigs);



    }
    
    public void periodic() {

        if (printCount++ > 10) {
      printCount = 0;
      // If any faults happen, print them out. Sticky faults will always be present if live-fault occurs
      f_fusedSensorOutOfSinc.refresh();
      sf_fusedSensorOutOfSinc.refresh();
    
      boolean anyFault = sf_fusedSensorOutOfSinc.getValue();
      if(anyFault) {
        System.out.println("A fault has occurred:");
        /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
        if(f_fusedSensorOutOfSinc.getValue()) {
          System.out.println("Fused sensor out of sync live-faulted");
        } else if (sf_fusedSensorOutOfSinc.getValue()) {
          System.out.println("Fused sensor out of sync sticky-faulted");
        }
      
      }

      /* Print out current position and velocity */
      fx_pos.refresh(); fx_vel.refresh();
      cc_pos.refresh(); cc_vel.refresh();
      var velocity = cc_vel.getValue();
      if (velocity > maxVelocity){
        maxVelocity = velocity;
      }
     // System.out.println("FX Position: " + fx_pos + " FX Vel: " + fx_vel);
     // System.out.println("CC Position: " + cc_pos + " CC Vel: " + cc_vel);
      System.out.println("Setpoint: " + m_positionSetpoint);
      System.out.println("degrees" + m_degrees);
      //System.out.println("Velocity " + maxVelocity);


     // SmartDashboard.putNumber("FX Position", fx_pos);

    }
  }

    public void stop() {
        m_motor.set(0);
    }


    public void raise(double raiseSpeed) {

        m_motor.set(raiseSpeed);
    }


    private double m_lastSpeed;

    // private double accelFF() {
    //     double currentVelocity = m_motor.getActiveTrajectoryVelocity();
    //     double accelConstant = .0000244140625;
    //     double accelFF = 0;

    //     if(currentVelocity - m_lastSpeed > 0) {
    //         accelFF = accelConstant * kMMAccel;
    //     } else if(currentVelocity - m_lastSpeed < 0){
    //         accelFF = -accelConstant * kMMAccel;
    //     }

    //     m_lastSpeed = m_motor.getActiveTrajectoryVelocity();
    //     return accelFF;
    // }

    //private double m_degrees;

    public void setToPositionSlow(double degrees) {
        m_degrees = degrees;
        m_motmag.Slot = 0;
        m_slow = true;

       // degrees = Math.max(Math.min(degrees, 64), -180);

        var position = degrees / 360.0 + kHorizontalPosition;
        m_positionSetpoint = position;

       // double gravityOffsetFF = ArmConstants.kMaxConeGravityFF * getGravityOffsetScalar(); NEED TO CONFIGURE MOTOR FOR THIS
        m_motor.setControl(m_motmag.withPosition(position));    //   / 360    
    }

    public void setToPosition(double degrees) {
        m_degrees = degrees;
        m_motmag.Slot = 0;
        m_slow = true;

       // degrees = Math.max(Math.min(degrees, 64), -180);

        var position = degrees / 360.0 + kHorizontalPosition;
        m_positionSetpoint = position;

       // double gravityOffsetFF = ArmConstants.kMaxConeGravityFF * getGravityOffsetScalar(); NEED TO CONFIGURE MOTOR FOR THIS
        m_motor.setControl(m_motmag.withPosition(position));    //   / 360
    }


    public Double setpoint() {
        return m_degrees;
    }

    public double currentPosition() {
        return m_CanCoder.getAbsolutePosition().getValue();
    }

    public double currentDegrees(){
      return (currentPosition() - kHorizontalPosition) * 360;
    }

    public boolean isSlow(){
        return m_slow;
    }

    // private double getGravityOffsetScalar() {
    //     double radians = java.lang.Math.toRadians(currentPosition());
    //     return java.lang.Math.cos(radians);
    // }


}
