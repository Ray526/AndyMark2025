// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constant;
import frc.robot.subsystems.Chassis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Move1M extends Command {
  private Chassis m_Chassis = new Chassis();

  private final PIDController pid = new PIDController(Constant.Chassis.kP, Constant.Chassis.kI, Constant.Chassis.kD);
  private double targetPos = 1.0;

  public Move1M() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the PID Setpoint(target)
    pid.setSetpoint(targetPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(m_Chassis.getCurrentPos());
    m_Chassis.tankDrive(output, output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Chassis.setMotorZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pid.atSetpoint()) {
      return true;
    }
    return false;
  }
}
