// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

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

  /** Called once at the beginning of the robot program. */
  public Robot() {

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor1.setInverted(false);
    m_leftMotor2.setInverted(false);
    m_rightMotor1.setInverted(true);
    m_rightMotor2.setInverted(true);
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
