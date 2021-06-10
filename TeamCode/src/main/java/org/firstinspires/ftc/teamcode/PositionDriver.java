package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robot.DriveTrain;
import org.firstinspires.ftc.robot_utilities.DashboardCorrections;
import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot_utilities.PositionController;
import org.firstinspires.ftc.robot_utilities.RotationController;
import org.firstinspires.ftc.robot_utilities.Vals;

@TeleOp(name = "PositionDriver")
public class PositionDriver extends OpMode {

    private FtcDashboard dashboard;
    private GamePadController gamepad;

    private RotationController rotationController;
    private PositionController positionController;

    boolean atSetPoint = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        gamepad = new GamePadController(gamepad1);

        rotationController = new RotationController(hardwareMap.get(BNO055IMU.class, "imu"));
        positionController = new PositionController(rotationController,
                new Motor(hardwareMap, "dl"),
                new Motor(hardwareMap, "dr"));
        positionController.reset();
    }

    @Override
    public void loop() {
        gamepad.update();

        if(gamepad.isXRelease()) {
            positionController.reset();
        }
        if(gamepad.isYRelease()) {
            positionController.updatePID();
        }

        atSetPoint = positionController.goStraight(Vals.test_travel_dist, Vals.drive_position_max_speed);
//        atSetPoint = positionController.rotateInPlace(Vals.rotate_target);


        telemetry.addData("Distance Traveled", positionController.driveTrain.getAverageDistance());
        telemetry.addData("Drive Speed", Vals.test_pDrive_val);
        telemetry.addData("At Set Point", atSetPoint);
    }
}
