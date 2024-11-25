// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class SwerveDrive {

    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    PigeonIMU gyro;
    //SwerveModule[] swerveModules;

    public SwerveDrive(){
        //
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(13.75), Units.inchesToMeters(14.125)), //front left
            new Translation2d(Units.inchesToMeters(13.75), Units.inchesToMeters(-14.125)), //front right
            new Translation2d(Units.inchesToMeters(-13.75), Units.inchesToMeters(14.125)), //back left
            new Translation2d(Units.inchesToMeters(-13.75), Units.inchesToMeters(-14.125)) //back right
        );
    }

    public void Drive(){
        ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(14), Units.inchesToMeters(4), Units.degreesToRadians(30));

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);
    }

}
