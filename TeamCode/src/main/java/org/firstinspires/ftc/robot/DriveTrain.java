package org.firstinspires.ftc.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;

import org.firstinspires.ftc.robot_utilities.Vals;

public class DriveTrain {

    public Motor driveLeft, driveRight;


    public DriveTrain(Motor driveLeft, Motor driveRight) {
        this.driveLeft = driveLeft;
        this.driveRight = driveRight;

        driveLeft.setRunMode(Motor.RunMode.VelocityControl);
        driveRight.setRunMode(Motor.RunMode.VelocityControl);
//        driveLeft.setInverted(true);
        updatePID();
    }

    public DriveTrain(Motor driveLeft, Motor driveRight, Motor.RunMode runMode) {
        this(driveLeft, driveRight);

        this.driveLeft.setRunMode(runMode);
        this.driveRight.setRunMode(runMode);
        updatePID();
    }

    public void updatePID() {
        driveLeft.setVeloCoefficients(Vals.drive_kp, Vals.drive_ki, Vals.drive_kd);
        driveRight.setVeloCoefficients(Vals.drive_kp, Vals.drive_ki, Vals.drive_kd);
        driveLeft.setPositionCoefficient(Vals.drive_kp);
        driveRight.setPositionCoefficient(Vals.drive_kp);
        driveLeft.setFeedforwardCoefficients(Vals.drive_ks, Vals.drive_kv);
        driveRight.setFeedforwardCoefficients(Vals.drive_ks, Vals.drive_kv);
    }

    public void stop() {
        setSpeed(0, 0);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        driveLeft.set(-leftSpeed);
        driveRight.set(rightSpeed);
    }

    public void setSpeedPositiveForward(double leftSpeed, double rightSpeed) {
        driveLeft.set(leftSpeed);
        driveRight.set(-rightSpeed);
    }

    public int[] getPosition() {
        return new int[]{driveLeft.getCurrentPosition(), driveRight.getCurrentPosition()};
    }

    public double[] getDistance() {
        return new double[]{driveLeft.getDistance(), -driveRight.getDistance()};
    }

    public double getAverageDistance() {
        return (driveLeft.getDistance() - driveRight.getDistance())/2;
    }

    public double[] getRevolutions() {
        return new double[]{driveLeft.encoder.getRevolutions(), driveRight.encoder.getRevolutions()};
    }

    public void resetEncoders() {
        driveLeft.resetEncoder();
        driveRight.resetEncoder();
    }



}
