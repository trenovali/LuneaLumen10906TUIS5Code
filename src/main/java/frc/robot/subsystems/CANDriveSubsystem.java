// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;
  private final DifferentialDrive drive;

  //private AHRS m_studicaGyro;
  private DifferentialDriveOdometry m_Odometry;
  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);
    /*try {
     m_studicaGyro = new AHRS(AHRS.NavXComType.kMXP_SPI); 
    } catch (RuntimeException ex) {
      System.out.println("Studica Gyro başlatilamadi: " + ex.getMessage());
    }

    if (m_studicaGyro != null) {
      m_studicaGyro.zeroYaw();
    }
  
    m_Odometry = new DifferentialDriveOdometry(getRotation2d(), null, null);*/

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

// ================= SOL MOTORLARIN AYARLARI =================
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.voltageCompensation(12);
    leftConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    leftConfig.inverted(true); // Sol tarafı ters çeviriyoruz ki pozitif değerler ileri götürsün

    // Sol lideri yapılandır
    leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Takipçi ayarı: Sol lideri takip etmesini söyle ve sol takipçiye yükle
    leftConfig.follow(leftLeader);
    leftFollower.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // ================= SAĞ MOTORLARIN AYARLARI =================
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.voltageCompensation(8.15);
    rightConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    rightConfig.inverted(false); // Sağ taraf düz kalıyor (varsayılan zaten false)

    // Sağ lideri yapılandır
    rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Takipçi ayarı: Sağ lideri takip etmesini söyle ve sağ takipçiye yükle
    rightConfig.follow(rightLeader);
    rightFollower.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   SmartDashboard.putData("şase", drive);
    SmartDashboard.putNumber("sol Lider",  leftLeader.getBusVoltage());
    SmartDashboard.putNumber("sag Lider",  rightLeader.getBusVoltage());
    SmartDashboard.putNumber("sol takipchi",  leftFollower.getBusVoltage());
    SmartDashboard.putNumber("sag takipchi",  rightFollower.getBusVoltage());
    SmartDashboard.putNumber("sol lider akim", leftLeader.getOutputCurrent());
    SmartDashboard.putNumber("sag lider akim", rightLeader.getOutputCurrent());
    SmartDashboard.putNumber("sol takipchi akim", leftFollower.getOutputCurrent());
    SmartDashboard.putNumber("sag takipchi akim", rightFollower.getOutputCurrent());
  }

  @Override
  public void periodic() {
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /*public Rotation2d getRotation2d() {
    if (m_studicaGyro != null) {
      return Rotation2d.fromDegrees(-m_studicaGyro.getYaw());
    }
    return new Rotation2d();
  }*/
}
