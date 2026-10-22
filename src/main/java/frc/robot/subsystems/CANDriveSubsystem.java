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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

// Logging
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault()
          .getTable("Drive")
          .getStructTopic("Robot Pose", Pose2d.struct)
          .publish();

  Pigeon2 pigeon;

  SparkMax leftFront;
  SparkMax leftRear;
  SparkMax rightFront;
  SparkMax rightRear;

  double testSpeed = 0;

  private final DifferentialDriveOdometry m_odometry;

  private final Field2d m_field = new Field2d();

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public CANDriveSubsystem() {
    leftFront = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftRear = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightFront = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightRear = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // Gyro
    pigeon = new Pigeon2(PIGEON_CAN_ID);
    StatusSignal<Angle> yaw = pigeon.getYaw();
    StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    // StatusSignal<Double> pigeonX = pigeon.getQuatX();
    // StatusSignal<Double> pigeonY = pigeon.getQuatY();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, yaw, yawVelocity);
    pigeon.optimizeBusUtilization();
    
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
    rightRear.configure(rightRearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFront.configure(rightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Odometey setup
     m_odometry = new DifferentialDriveOdometry(
                pigeon.getRotation2d(), 0, 0);

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

    // AutoBuilder.configure(
    //     this::getPose,
    //     this::setPose,
    //     () ->
    //         kinematics.toChassisSpeeds(
    //             new DifferentialDriveWheelSpeeds(
    //                 getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
    //     (ChassisSpeeds speeds) -> runClosedLoop(speeds),
    //     new PPLTVController(0.02, maxSpeedMetersPerSec),
    //     ppConfig,
    //     () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
    //     this);
    // Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
    //     });
    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    //     });

    // // Configure SysId
    // sysId =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //             null,
    //             null,
    //             null,
    //             (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //             (voltage) -> runOpenLoop(voltage.in(Volts), voltage.in(Volts)), null, this));

    // PathPlanner AutoBuilder Config

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
                m_odometry::getPoseMeters, 
                this::resetOdometry, 
                this::getChassisSpeeds,
                this::driveRobotRelative, 
                new PPLTVController(VecBuilder.fill(0.03, 0.03, 5), 
                                    VecBuilder.fill(0.1, 60),
                                    0.02,
                                    5.45), 
                
                config, () -> {
                                                                                                              // Boolean supplier that controls when the path will be mirrored for the red alliance
                                                                                                              // This will flip the path being followed to the red side of the field.
                                                                                                              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                                                                                                              var alliance = DriverStation.getAlliance();
                                                                                                              if (alliance.isPresent()) {
                                                                                                                return alliance.get() == DriverStation.Alliance.Red;
                                                                                                              }
                                                                                                              return false;
                                                                                                            }, this);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    PathfindingCommand.warmupCommand().schedule();
    

  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
    Logger.recordOutput("Drive/SpeedInput", speed);
    Logger.recordOutput("Drive/RotationInput", rotation);
  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public Command arcadeDriveCommand(double speed, double rotation) {
    return run(() -> {m_drivetrain.arcadeDrive(speed, rotation);
    Logger.recordOutput("Drive/SpeedInput", speed);
    Logger.recordOutput("Drive/RotationInput", rotation);});
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
      DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);

      tankDrive(wheelSpeeds.leftMetersPerSecond,wheelSpeeds.rightMetersPerSecond);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    m_drivetrain.tankDrive(leftSpeed,rightSpeed);
    Logger.recordOutput("Drive/LeftSpeedInput", leftSpeed);
    Logger.recordOutput("Drive/RightSpeedInput", rightSpeed);
  }

  public ChassisSpeeds getChassisSpeeds() {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds((leftFront.getEncoder().getVelocity()*(DriveConstants.kWheelRadius * 2.0 * Math.PI)/60)*DriveConstants.DRIVE_GEAR_RATIO,
            (rightFront.getEncoder().getVelocity()*(DriveConstants.kWheelRadius * 2.0 * Math.PI)/60)*DriveConstants.DRIVE_GEAR_RATIO);
    ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(wheelSpeeds);

    return speeds;
  }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pigeon.getRotation2d(), leftFront.getEncoder().getPosition() * DriveConstants.DRIVE_GEAR_RATIO * (DriveConstants.kWheelRadius * 2.0 * Math.PI),
            rightFront.getEncoder().getPosition() * DriveConstants.DRIVE_GEAR_RATIO * (DriveConstants.kWheelRadius * 2.0 * Math.PI), pose);
    }


    public Command resetOdometryCommand(Pose2d pose) {
        return run(() -> {resetOdometry(pose);});
    }

  @Override
  public void periodic() {
    m_odometry.update(pigeon.getRotation2d(),
            leftFront.getEncoder().getPosition() * DriveConstants.DRIVE_GEAR_RATIO * (DriveConstants.kWheelRadius * 2.0 * Math.PI),
            rightFront.getEncoder().getPosition() * DriveConstants.DRIVE_GEAR_RATIO * (DriveConstants.kWheelRadius * 2.0 * Math.PI));

    // if((leftFront.getEncoder().getVelocity()*(DriveConstants.kWheelRadius * 2.0 * Math.PI)/60)*DriveConstants.DRIVE_GEAR_RATIO > testSpeed){
    //   testSpeed = (leftFront.getEncoder().getVelocity()*(DriveConstants.kWheelRadius * 2.0 * Math.PI)/60)*DriveConstants.DRIVE_GEAR_RATIO;
    //   System.out.println(testSpeed);
    // }

    // System.out.println((leftFront.getEncoder().getVelocity()*(DriveConstants.kWheelRadius * 2.0 * Math.PI)/60)*DriveConstants.DRIVE_GEAR_RATIO);

    m_field.setRobotPose(m_odometry.getPoseMeters());

    posePublisher.accept(m_odometry.getPoseMeters());
    Logger.recordOutput("Pigeon/Yaw", pigeon.getYaw().getValue());
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }
}