// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team1891.common.drivetrains.swervemodules.SteerController;
import frc.team1891.common.drivetrains.swervemodules.SwerveModule;

public class FalconSteerControllerModified implements SteerController {
    private final WPI_TalonFX steerMotor;
    private final CANCoder encoder;
    private final double steeringGearRatio;

    private final double cancoderOffsetDegrees;

    // TODO: Neutral mode and encoder invert params
    // TODO: Don't take object as param if configuring.  Trust user to configure if object is passed.  Take CAN IDs instead

    // Uses the defaults based on an MK4i module
    public FalconSteerControllerModified(WPI_TalonFX steerMotor, CANCoder encoder, double encoderOffsetDegrees) {
        this(steerMotor, encoder, (150/7.), encoderOffsetDegrees, .3, 0, 0, 0);
    }

    public FalconSteerControllerModified(WPI_TalonFX steerMotor, CANCoder encoder, double steeringGearRatio, double encoderOffsetDegrees, double kP, double kI, double kD, double kFF) {
        this.steerMotor = steerMotor;
        this.encoder = encoder;
        this.steeringGearRatio = steeringGearRatio;
        this.cancoderOffsetDegrees = encoderOffsetDegrees;

        configureSteerMotor(kP, kI, kD, kFF);
    }

    private void configureSteerMotor(double kP, double kI, double kD, double kFF) {
        steerMotor.configFactoryDefault();
        final SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                true,
                25,
                40,
                .1);

        final TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.supplyCurrLimit = angleSupplyLimit;
        configuration.slot0.kP = kP;
        configuration.slot0.kI = kI;
        configuration.slot0.kD = kD;
        configuration.slot0.kF = kFF;
        steerMotor.configAllSettings(configuration);
        steerMotor.setInverted(true);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        calibrateEncoders();
    }

    public void calibrateEncoders() {
        double initialPosition = steerMotor.getSelectedSensorPosition();
        double absolutePosition = SwerveModule.degreesToMotorEncoderTicks(encoder.getAbsolutePosition() - cancoderOffsetDegrees, steeringGearRatio, 2048);
        ErrorCode errorCode = steerMotor.setSelectedSensorPosition(absolutePosition, 0, 30);
        System.out.printf("INFO: FalconSteerController: Calibrated Encoders. (ErrorCode: %s)\n", errorCode.name());
        System.out.printf("\tPrevious sensor position: %f  - New sensor position: %f\n", initialPosition, steerMotor.getSelectedSensorPosition());
    }

    @Override
    public void drive(SwerveModuleState state) {
        Rotation2d angle = state.angle;
        double angleDegrees = placeInAppropriate0To360Scope(getDegrees(), angle.getDegrees());

        steerMotor.set(ControlMode.Position, SwerveModule.degreesToMotorEncoderTicks(angleDegrees, steeringGearRatio, 2048));
    }

    @Override
    public void stop() {
        steerMotor.stopMotor();
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(SwerveModule.motorEncoderTicksToRadians(steerMotor.getSelectedSensorPosition(), steeringGearRatio, 2048));
    }

    @Override
    public double getRadians() {
        return SwerveModule.motorEncoderTicksToRadians(steerMotor.getSelectedSensorPosition(), steeringGearRatio, 2048);
    }

    @Override
    public double getDegrees() {
        return SwerveModule.motorEncoderTicksToDegrees(steerMotor.getSelectedSensorPosition(), steeringGearRatio, 2048);
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
}

