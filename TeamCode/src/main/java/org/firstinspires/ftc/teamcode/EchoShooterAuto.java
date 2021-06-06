package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot.DriveTrain;
import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot.Hitter;
import org.firstinspires.ftc.robot.Intake;
import org.firstinspires.ftc.robot.WobbleSystem;
import org.firstinspires.ftc.robot_utilities.RotationController;
import org.firstinspires.ftc.robot_utilities.Vals;
import org.firstinspires.ftc.robot_utilities.VisionController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="EchoShooterAuto")
public class EchoShooterAuto extends OpMode {

    ElapsedTime elapsedTime;
    VisionController visionController;
    RotationController rotationController;

    private DriveTrain driveTrain;
    private FlyWheel flywheel;
    private WobbleSystem wobbleSystem;
    private Hitter hitter;
    private Intake intake;

    boolean notStarted = true;
    double driveSpeed = 0.4;
    int shotsFired = 0;

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvInternalCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        visionController = new VisionController(phoneCam);

        rotationController = new RotationController(hardwareMap.get(BNO055IMU.class, "imu"));
        rotationController.resetAngle();

        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"), new Motor(hardwareMap, "dr"));
        driveTrain.resetEncoders();
        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE), telemetry);
        wobbleSystem = new WobbleSystem(new Motor(hardwareMap, "wobbleArmMotor"), hardwareMap.servo.get("wobbleArmServo"));
        wobbleSystem.hand_close();
        hitter = new Hitter(hardwareMap.servo.get("sv"));
        intake = new Intake(new Motor(hardwareMap, "in1"),  new Motor(hardwareMap, "in2"));
    }

    @Override
    public void loop() {
        if(notStarted) {
            elapsedTime.reset();

            notStarted = false;
            return;
        }

        if(elapsedTime.seconds() < 1.2) {
            double rotatePower = rotationController.rotate(-10);
            double leftPower = -rotatePower + driveSpeed;
            double rightPower = rotatePower + driveSpeed;
            driveTrain.setSpeedPositiveForward(leftPower, rightPower);
        } else if(shotsFired == 0) {
            telemetry.addData("In Shots Fired", 0);
            driveTrain.stop();
            flywheel.on_slow();
            if(flywheel.isReadySlow()) {
                hitter.hitFullMotion(0.7);
                shotsFired++;
                rotationController.resetAngle();
            }
        } else if(shotsFired == 1) {
            telemetry.addData("In Shots Fired", 1);
            flywheel.on_slow();

            double rotatePower = rotationController.rotate(5);
            double leftPower = -rotatePower;
            double rightPower = rotatePower;
            driveTrain.setSpeedPositiveForward(leftPower, rightPower);

            if(flywheel.isReadySlow() && rotationController.atRotation()) {
                shotsFired++;
                hitter.hitFullMotion(0.7);
                rotationController.resetAngle();
                driveTrain.stop();
            }

        } else if(shotsFired == 2) {
            telemetry.addData("In Shots Fired", 2);
            flywheel.on_slow();

            double rotatePower = rotationController.rotate(5);
            double leftPower = -rotatePower;
            double rightPower = rotatePower;
            driveTrain.setSpeedPositiveForward(leftPower, rightPower);

            if(flywheel.isReadySlow() && rotationController.atRotation()) {
                shotsFired++;
                hitter.hitFullMotion(0.7);
                rotationController.resetAngle();
                driveTrain.stop();
            }

        } else {
            requestOpModeStop();
        }

        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Rotation Angle Radians", rotationController.getAngleRadians());
        telemetry.addData("Rotation At Rotation", rotationController.atRotation());
        telemetry.addData("DriveLeft Speed", driveTrain.driveLeft.get());
        telemetry.addData("DriveRight Speed", driveTrain.driveRight.get());

    }
}
