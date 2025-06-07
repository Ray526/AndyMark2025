// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);

  private final TalonSRX m_leftMotor1 = new TalonSRX(0);
  private final TalonSRX m_leftMotor2 = new TalonSRX(1);
  private final TalonSRX m_rightMotor1 = new TalonSRX(2);
  private final TalonSRX m_rightMotor2 = new TalonSRX(3);

  private final CANcoder m_CanCoder1 = new CANcoder(11); // left cancoder
  private final CANcoder m_CanCoder2 = new CANcoder(12); // right cancoder
  private final CANcoderConfiguration leftConfig = new CANcoderConfiguration();
  private final CANcoderConfiguration rightConfig = new CANcoderConfiguration();

  /** Called once at the beginning of the robot program. */
  public Robot() {

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
  }

  // Called every 20ms, no matter the robot mode (disabled, autonomous, teleop, or test).
  // use this for code that needs to run constantly, such as updating sensors or dashboards.
  @Override
  public void robotPeriodic() {
    // Publish data to SmartDashboard (cuz we get the CANcoder in double,
    // we use SmartDashboard.putNumber()
    // other common methods: putData(), putString(), putBoolean())
    // the String key("left/right CAMcoder") will display on SmartDashboard(a block)
    // btw we can use Shuffleboard and Elastic to check the data
    SmartDashboard.putNumber("left CANcoder", m_CanCoder1.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("right CANcoder", m_CanCoder2.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void teleopPeriodic() {
    // There are two types of control for the AndyMark drivetrain:
    // Ctrl + click to navigate to each method
    arcadeDrive(controller.getLeftY(), controller.getLeftX());
    tankDrive(controller.getLeftY(), controller.getRightY());
  }

  // use 1 joystick to control the chassis
  // forward/backward for speed
  // left/right for turning
  private void arcadeDrive(double throttle, double rotation) {
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
  private void tankDrive(double leftSpeed, double rightSpeed) {
    // simply give speed to motors
    m_leftMotor1.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    m_leftMotor2.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    m_rightMotor1.set(TalonSRXControlMode.PercentOutput, rightSpeed);
    m_rightMotor2.set(TalonSRXControlMode.PercentOutput, rightSpeed);
  }
}
