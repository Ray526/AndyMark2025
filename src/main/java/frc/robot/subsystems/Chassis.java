// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant;

public class Chassis extends SubsystemBase {

  private final WPI_TalonSRX m_leftMotor1 = new WPI_TalonSRX(0); // front
  private final WPI_TalonSRX m_leftMotor2 = new WPI_TalonSRX(1); // rear
  private final WPI_TalonSRX m_rightMotor1 = new WPI_TalonSRX(2); // front
  private final WPI_TalonSRX m_rightMotor2 = new WPI_TalonSRX(3); // rear

  private final CANcoder m_CanCoder1 = new CANcoder(11); // left cancoder
  private final CANcoder m_CanCoder2 = new CANcoder(12); // right cancoder
  private final CANcoderConfiguration leftConfig = new CANcoderConfiguration();
  private final CANcoderConfiguration rightConfig = new CANcoderConfiguration();

  public Chassis() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor1.setInverted(false);
    m_leftMotor2.setInverted(false);
    m_rightMotor1.setInverted(true);
    m_rightMotor2.setInverted(true);

    // left CANcoder config
    leftConfig.MagnetSensor.MagnetOffset = 0.0;
    leftConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // CANcoder config
    rightConfig.MagnetSensor.MagnetOffset = 0.0;
    rightConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    // apply config to CANcoders
    m_CanCoder1.getConfigurator().apply(leftConfig);
    m_CanCoder2.getConfigurator().apply(rightConfig);

    // reset CANcoders position to 0 (for PID calculate)
    m_CanCoder1.setPosition(0.0);
    m_CanCoder2.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // Publish data to SmartDashboard (cuz we get the CANcoder in double,
    // we use SmartDashboard.putNumber()
    // other common methods: putData(), putString(), putBoolean())
    // the String key("left/right CAMcoder") will display on SmartDashboard(a block)
    // btw we can use Shuffleboard and Elastic to check the data
    SmartDashboard.putNumber("left CANcoder", m_CanCoder1.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("right CANcoder", m_CanCoder2.getAbsolutePosition().getValueAsDouble());
  }

  // use 1 joystick to control the chassis
  // forward/backward for speed
  // left/right for turning
  public void arcadeDrive(double throttle, double rotation) {
    // imagine if a chassis is trying to turn right,
    // its leftMotors need more power than right
    double leftSpeed = throttle + rotation;
    double rightSpeed = throttle - rotation;

    // clamp speed between -1.0 and 1.0
    leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
    rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));

    // give the speed to motors
    m_leftMotor1.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    m_leftMotor2.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    m_rightMotor1.set(TalonSRXControlMode.PercentOutput, rightSpeed);
    m_rightMotor2.set(TalonSRXControlMode.PercentOutput, rightSpeed);
  }

  // Use 2 joysticks
  // Left joystick Y controls the left motors
  // Right joystick Y controls the right motors
  public void tankDrive(double leftSpeed, double rightSpeed) {
    // simply give speed to motors
    m_leftMotor1.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    m_leftMotor2.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    m_rightMotor1.set(TalonSRXControlMode.PercentOutput, rightSpeed);
    m_rightMotor2.set(TalonSRXControlMode.PercentOutput, rightSpeed);
  }

  public double getCurrentPos() {
    // Get the rotation by averaging the values from two CANcoders to improve accuracy
    double rotation = (m_CanCoder1.getPosition().getValueAsDouble() + m_CanCoder2.getPosition().getValueAsDouble()) / 2;
    // Convert rotation to position (Meters)
    double position = rotation * Constant.Chassis.wheelDiameter * Math.PI;
    return position;
  }

  public void setMotorZero() {
    m_leftMotor1.set(0);
    m_leftMotor2.set(0);
    m_rightMotor1.set(0);
    m_rightMotor2.set(0);
  }
}
