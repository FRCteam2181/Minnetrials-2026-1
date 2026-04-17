// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANRollerSubsystem;
import java.util.function.DoubleSupplier;

// Command to run the roller with joystick inputs
public class RollerSpitterCommand extends Command {
  private final DoubleSupplier topForward;
  private final DoubleSupplier topReverse;
  private final DoubleSupplier bottomForward;
  private final DoubleSupplier bottomReverse;
  // private final CANRollerSubsystem rollerSubsystem;
  private final CANRollerSubsystem rollerSubsystem;

  public RollerSpitterCommand(
      DoubleSupplier topForward, DoubleSupplier topReverse, DoubleSupplier bottomForward, DoubleSupplier bottomReverse, CANRollerSubsystem rollerSubsystem) {
    this.topForward = topForward;
    this.topReverse = topReverse;
    this.bottomForward = bottomForward;
    this.bottomReverse = bottomReverse;
    this.rollerSubsystem = rollerSubsystem;

    addRequirements(this.rollerSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    // Run the roller motor at the desired speed
    rollerSubsystem.runRollerSpitter(topForward.getAsDouble(), topReverse.getAsDouble(), bottomForward.getAsDouble(), bottomReverse.getAsDouble());
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}