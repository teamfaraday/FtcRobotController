package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot.DriveTrain;
import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot.Hitter;
import org.firstinspires.ftc.robot.Intake;
import org.firstinspires.ftc.robot.WobbleSystem;
import org.firstinspires.ftc.robot_utilities.RotationController;
import org.firstinspires.ftc.robot_utilities.VisionController;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class EchoRedRightAuto extends OpMode {

    ElapsedTime elapsedTime;
    VisionController visionController;
    RotationController rotationController;
    boolean notStarted = true;
    int ringStackSize = -1;
    int shotsFired = 0;
    int ticks = 0;
    private DriveTrain driveTrain;
    private FlyWheel flywheel;
    private WobbleSystem wobbleSystem;
    private Hitter hitter;
    private Intake intake;

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
        intake = new Intake(new Motor(hardwareMap, "in1"), new Motor(hardwareMap, "in2"));

    }

    @Override
    public void loop() {

        if (notStarted) {
            ringStackSize = visionController.getRingPosition();
            elapsedTime.reset();

            notStarted = false;
            return;
        }

        double driveSpeed = 0.4;

        switch (ringStackSize) {

            case 0:

                //No angle.

                //Drive forward
                if (elapsedTime.seconds() < 1.7) {

                    double rotatePower = rotationController.rotate(0);

                    double leftPower = -rotatePower;
                    double rightPower = rotatePower;

                    leftPower += driveSpeed;
                    rightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(leftPower, rightPower);

                }

                elapsedTime.reset();

                //Shoot 3 high goals.
                if (ticks < 3 && elapsedTime.seconds() < 1.7) {

                    flywheel.on();
                    hitter.hitFullMotion(0.7);
                    ticks++;

                }

                elapsedTime.reset();

                //Deliver wobble goal to A.
                if (elapsedTime.seconds() < 1.7) {
                    double rotatePower = rotationController.rotate(22);

                    double leftPower = -rotatePower;
                    double rightPower = rotatePower;

                    leftPower += driveSpeed;
                    rightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(leftPower, rightPower);
                } else if (elapsedTime.seconds() < 3) {
                    driveTrain.stop();
                } else if (elapsedTime.seconds() < 4.5) {
                    telemetry.addData("Wobble Arm Location", wobbleSystem.wobbleArm.getCurrentPosition());
                    telemetry.addData("Wobble Arm Speed", wobbleSystem.wobbleArm.get());
                    telemetry.addData("Wobble Hand Pos", wobbleSystem.wobbleHand.getPosition());
                    telemetry.update();
                    wobbleSystem.arm_down();
                } else if (elapsedTime.seconds() < 5.5) {
                    wobbleSystem.hand_open();
                }

                elapsedTime.reset();

                //Park
                if (elapsedTime.seconds() < 1.7) {
                    double parkRotatePower = rotationController.rotate(0);

                    double parkLeftPower = -parkRotatePower;
                    double parkRightPower = parkRotatePower;

                    parkLeftPower += driveSpeed;
                    parkRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(parkLeftPower, parkRightPower);

                } else {

                    requestOpModeStop();

                }

                break;

            case 1:
                //No angle.

                //Drive forward
                if (elapsedTime.seconds() < 1.7) {

                    double rotatePower = rotationController.rotate(0);

                    double leftPower = -rotatePower;
                    double rightPower = rotatePower;

                    leftPower += driveSpeed;
                    rightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(leftPower, rightPower);

                }

                elapsedTime.reset();

                //Shoot 3 high goals.
                if (ticks < 3 && elapsedTime.seconds() < 1.7) {

                    flywheel.on();
                    hitter.hitFullMotion(0.7);
                    ticks++;

                }

                elapsedTime.reset();

                //Deliver wobble goal to B.
                if (elapsedTime.seconds() < 3) {
                    double rotatePower = rotationController.rotate(22);

                    double leftPower = -rotatePower;
                    double rightPower = rotatePower;

                    leftPower += driveSpeed;
                    rightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(leftPower, rightPower);
                } else if (elapsedTime.seconds() < 5.3) {
                    driveTrain.stop();
                } else if (elapsedTime.seconds() < 4.5) {
                    telemetry.addData("Wobble Arm Location", wobbleSystem.wobbleArm.getCurrentPosition());
                    telemetry.addData("Wobble Arm Speed", wobbleSystem.wobbleArm.get());
                    telemetry.addData("Wobble Hand Pos", wobbleSystem.wobbleHand.getPosition());
                    telemetry.update();
                    wobbleSystem.arm_down();
                } else if (elapsedTime.seconds() < 6.5) {
                    wobbleSystem.hand_open();
                }

                elapsedTime.reset();

                //Park
                if (elapsedTime.seconds() < 1.7) {
                    double parkRotatePower = rotationController.rotate(0);

                    double parkLeftPower = -parkRotatePower;
                    double parkRightPower = parkRotatePower;

                    parkLeftPower += driveSpeed;
                    parkRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(parkLeftPower, parkRightPower);

                } else {

                    requestOpModeStop();

                }

                break;

            case 4:
                //No angle.

                //Drive forward
                if (elapsedTime.seconds() < 1.7) {

                    double rotatePower = rotationController.rotate(0);

                    double leftPower = -rotatePower;
                    double rightPower = rotatePower;

                    leftPower += driveSpeed;
                    rightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(leftPower, rightPower);

                }

                elapsedTime.reset();

                //Shoot 3 high goals.
                if (ticks < 3 && elapsedTime.seconds() < 1.7) {

                    flywheel.on();
                    hitter.hitFullMotion(0.7);
                    ticks++;

                }

                elapsedTime.reset();

                //Deliver wobble goal to C.
                if (elapsedTime.seconds() < 5) {
                    double rotatePower = rotationController.rotate(22);

                    double leftPower = -rotatePower;
                    double rightPower = rotatePower;

                    leftPower += driveSpeed;
                    rightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(leftPower, rightPower);
                } else if (elapsedTime.seconds() < 8) {
                    driveTrain.stop();
                } else if (elapsedTime.seconds() < 9.5) {
                    telemetry.addData("Wobble Arm Location", wobbleSystem.wobbleArm.getCurrentPosition());
                    telemetry.addData("Wobble Arm Speed", wobbleSystem.wobbleArm.get());
                    telemetry.addData("Wobble Hand Pos", wobbleSystem.wobbleHand.getPosition());
                    telemetry.update();
                    wobbleSystem.arm_down();
                } else if (elapsedTime.seconds() < 11.5) {
                    wobbleSystem.hand_open();
                }

                elapsedTime.reset();

                //Park
                if (elapsedTime.seconds() < 1.7) {
                    double parkRotatePower = rotationController.rotate(0);

                    double parkLeftPower = -parkRotatePower;
                    double parkRightPower = parkRotatePower;

                    parkLeftPower += driveSpeed;
                    parkRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(parkLeftPower, parkRightPower);

                } else {

                    requestOpModeStop();

                }

                break;

            default:
                stop();
        }

    }
}
