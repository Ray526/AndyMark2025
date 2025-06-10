// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Teleop;
import frc.robot.subsystems.Chassis;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final CommandXboxController controller = new CommandXboxController(0);
  private Chassis m_Chassis = new Chassis();
  private Teleop teleop = new Teleop(controller);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_Chassis.setDefaultCommand(teleop);
  }

  // Called every 20ms, no matter the robot mode (disabled, autonomous, teleop, or
  // test).
  // use this for code that needs to run constantly, such as updating sensors or
  // dashboards.
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledPeriodic() {
  }
}
