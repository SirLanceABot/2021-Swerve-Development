// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
  //FIXME Change Wheel Radius and EncoderResolution
  public static final double kInchesToMeters = 0.0254;
  private static final double kWheelRadius = 2 * kInchesToMeters;
  private static final int kEncoderResolution = 2048;
  // FIXME Use these variables probably after we removed in a merge conflict
  // private static final double kWheelRadiusInches = 2;
  // private static final double kInchesToMeters = 0.0254;
  // private static final double kWheelRadiusMeters = kWheelRadiusInches * kInchesToMeters;
  // private static final int kDriveMotorEncoderResolution = 2048;
  // private static final int kTurningMotorEncoderResolution = 4096;
  // private static final double kDriveMotorGearRatio = 8.14;
  // private static final double kTurningMotorGearRatio = 12.8;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  //FIXME Convert to Talon FX
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  //FIXME Convert to Talon FX
  // private Encoder m_driveEncoder;
  private final CANCoder m_turningEncoder; //= new CANCoder();

  private final int m_turningEncoderOffset;

  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  //FIXME: Gains are for example purposes only - must be determined for your own robot!
  //First parameter is static gain (how much voltage it takes to move)
  //Second parameters is veloctiy gain (how much additional speed you get per volt)
  
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(Constants.SwerveModuleConstants smc)
  {
    m_driveMotor = new TalonFX(smc.driveMotorChannel);
    m_turningEncoder = new CANCoder(smc.turningMotorEncoder);
    m_turningMotor = new TalonFX(smc.turningMotorChannel);

    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    configTalon(m_driveMotor, smc.driveMotorInverted);
    configTalon(m_turningMotor, false);

    m_turningEncoderOffset = smc.turningMotorEncoderOffset;

    // resetTurningMotorEncoder();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static void configTalon(TalonFX motor, boolean inverted)
    {
        motor.configFactoryDefault();
        motor.setInverted(inverted);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configForwardSoftLimitEnable(false);
        motor.configReverseSoftLimitEnable(false);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); //TODO Convert to newer config API
        // motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
        // motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
        // motor.configOpenloopRamp(openLoopRamp);
        motor.configVoltageCompSaturation(11);
    }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDrivingEncoderRate(), new Rotation2d(getTurningEncoderPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getDrivingEncoderRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getTurningEncoderPosition(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    //FIXME Convert to Talon FX
    m_driveMotor.set(ControlMode.PercentOutput, driveOutput + driveFeedforward);
    m_turningMotor.set(ControlMode.PercentOutput, turnOutput + turnFeedforward);
  }

  public double getDrivingEncoderRate()
  {
    double velocity = (((m_driveMotor.getSelectedSensorVelocity() * 10) / kEncoderResolution) / 8.14) * (2 * Math.PI * kWheelRadius);
    // FIXME Units conversion?
    System.out.println(m_driveMotor.getDeviceID() + " " + velocity);
    return velocity;
  }

  private double getTurningEncoderPosition()
  {
    return m_turningEncoder.getAbsolutePosition() - m_turningEncoderOffset; 
  }

  // FIXME Fix reset turning motor encoder
  public void resetTurningMotorEncoder()
  {
    // m_turningEncoder.setPosition(m_turningEncoderOffset);
  }
}
