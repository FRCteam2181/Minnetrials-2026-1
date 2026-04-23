// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkParameters;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class CANDriveSubsystem extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  DifferentialDrive m_drivetrain;

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public CANDriveSubsystem() {
    SparkMax leftFront = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    SparkMax leftRear = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    SparkMax rightFront = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    SparkMax rightRear = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);
    
    // Defining CONFIGS
    SparkMaxConfig leftFrontConfig = new SparkMaxConfig();
    SparkMaxConfig leftRearConfig = new SparkMaxConfig();
    SparkMaxConfig rightFrontConfig = new SparkMaxConfig();
    SparkMaxConfig rightRearConfig = new SparkMaxConfig();

    leftFrontConfig
      .smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
      .voltageCompensation(12)
      .inverted(false);

    leftRearConfig
      .smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
      .voltageCompensation(12)
      .inverted(true)
      .follow(LEFT_LEADER_ID);
    
    rightFrontConfig
      .smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
      .voltageCompensation(12)
      .inverted(true);

    rightRearConfig
      .smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
      .voltageCompensation(12)
      .inverted(false)
      .follow(RIGHT_LEADER_ID);


    // Apply Configs

    leftFront.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftRear.configure(leftRearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightRear.configure(leftRearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFront.configure(rightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
     *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
    // leftFront.configPeakCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    // rightFront.configPeakCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);


    // // Set the rear motors to follow the front motors.
    // leftRear.follow(leftFront);
    // rightRear.follow(rightFront);

    // // Invert the left side so both side drive forward with positive motor outputs
    // leftFront.setInverted(false);
    // rightFront.setInverted(true);
    // leftRear.setInverted(false);
    // rightRear.setInverted(true);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }
}