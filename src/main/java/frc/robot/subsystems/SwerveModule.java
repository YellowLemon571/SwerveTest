// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  private static final int kEncoderResolution = 4096;

  private final PWMVictorSPX m_driveMotor;
  private final PWMVictorSPX m_turningMotor;

  private final Encoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Drivetrain.kMaxAngularSpeed, 2 * Math.PI));

  private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedForward = new SimpleMotorFeedforward(1, 0.5);

  /** Creates a new SwerveModule. */
  public SwerveModule(
    int driveMotorChannel,
    int turningMotorChannel,
    int turningEncoderChannelA,
    int turningEncoderChannelB
  ) {
    m_driveMotor = new PWMVictorSPX(0);
    m_turningMotor = new PWMVictorSPX(1);

    m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
    m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);
  }

  public SwerveModuleState getState() {
    double driveRate = m_driveMotor.get() * Drivetrain.kMaxSpeed;
    return new SwerveModuleState(driveRate, new Rotation2d(m_turningEncoder.get()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

    double driveRate = m_driveMotor.get() * Drivetrain.kMaxSpeed;
    final double driveOutput = m_drivePIDController.calculate(driveRate, state.speedMetersPerSecond);

    final double driveFeedForward = m_driveFeedForward.calculate(state.speedMetersPerSecond);

    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

    final double turnFeedForward = m_turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedForward);
    m_turningMotor.setVoltage(turnOutput + turnFeedForward);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive Speed", m_driveMotor.get() * Drivetrain.kMaxSpeed);
    SmartDashboard.putNumber("Rotation Speed", new Rotation2d(m_turningEncoder.get()).getDegrees());
  }
}
