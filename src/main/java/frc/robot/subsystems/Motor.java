// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
  private CANSparkMax _m = new CANSparkMax(3, MotorType.kBrushless);
  private RelativeEncoder _enc;
  private SparkMaxPIDController _pid;

  private RobotController _con = RobotController.getInstance();

  public double kRatio = 40.; // turret
  public double kRotations = 3.;
  public double kSpeed = 0.15;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kMaxRPM;

  enum EncoderState {
    FORWARD,
    BACKWARD,
    STOP,
    VELOCITY
  }

  private EncoderState _state = EncoderState.STOP;
  /** Creates a new Motor. */
  public Motor() {
    _m.restoreFactoryDefaults();

    _enc = _m.getEncoder();
    _pid = _m.getPIDController();

    kP = 0.00001;
    kI = 0;
    kD = 0.0001;
    kIz = 0;
    kFF = 0.0000875; // Standalone: 0.000168

    kMaxOutput = 1;
    kMinOutput = -1;
    kMaxRPM = 500; // 5310 is actual max: just don't want it to be obscenely fast for now

    _pid.setP(kP);
    _pid.setI(kI);
    _pid.setD(kD);
    _pid.setIZone(kIz);
    _pid.setFF(kFF);
    _pid.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Max RPM", kMaxRPM);

    zero();
  }

  public void controller() {
    if (_con.getXButtonPressed()) {
      _state = EncoderState.STOP;
    } else if (_con.getAButtonPressed()) {
      _state = EncoderState.FORWARD;
    } else if (_con.getBButtonPressed()) {
      _state = EncoderState.BACKWARD;
    } else if (_con.getYButtonPressed()) {
      _state = EncoderState.VELOCITY;
    }
  }

  public void pidLoop() {
    switch (_state) {
      case STOP:
        zero();
        break;
      case FORWARD:
        encoderRun(1);
        break;
      case BACKWARD:
        encoderRun(-1);
        break;
      case VELOCITY:
        run();
        break;
      default:
        break;
    }
  }

  public void zero() {
    _m.set(0);
    _enc.setPosition(0);
  }

  public void encoderRun(double mult) {
    if (mult * _enc.getPosition() < kRatio * kRotations) {
      _m.set(mult * kSpeed);
    } else {
      zero();
      _state = EncoderState.STOP;
    }
  }

  public void run() {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rpm = SmartDashboard.getNumber("Max RPM", 0);

    if((p != kP)) { _pid.setP(p); kP = p; }
    if((i != kI)) { _pid.setI(i); kI = i; }
    if((d != kD)) { _pid.setD(d); kD = d; }
    if((iz != kIz)) { _pid.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { _pid.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      _pid.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if ((rpm != kMaxRPM)) { kMaxRPM = rpm; }

    double point = _con.getLeftYAxis() * kMaxRPM;
    _pid.setReference(point, ControlType.kVelocity);

    SmartDashboard.putNumber("Target Velocity", point);
    SmartDashboard.putNumber("Actual Velocity", _enc.getVelocity());
    SmartDashboard.putNumber("Error", point - _enc.getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
