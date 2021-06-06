package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "EchoBlueRightAuto", group = "Echo Autos")
public class EchoBlueRightAuto extends OpMode {

    ElapsedTime elapsedTime;
    VisionController visionController;
    RotationController rotationController;

    private DriveTrain driveTrain;
    private FlyWheel flywheel;
    private WobbleSystem wobbleSystem;
    private Hitter hitter;
    private Intake intake;

    boolean notStarted = true;
    int ringStackSize = -1;


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
            ringStackSize = visionController.getRingPosition();
            elapsedTime.reset();

            notStarted = false;
            return;
        }

        double driveSpeed = 0.4;

        switch(ringStackSize) {
            case 0:
                if(elapsedTime.seconds() < 1.7) {
                    double rotatePower = rotationController.rotate(22);

                    double leftPower = -rotatePower;
                    double rightPower = rotatePower;

                    leftPower += driveSpeed;
                    rightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(leftPower, rightPower);
                } else if(elapsedTime.seconds() < 3) {
                    driveTrain.stop();
                } else if(elapsedTime.seconds() < 4.5) {
                    telemetry.addData("Wobble Arm Location", wobbleSystem.wobbleArm.getCurrentPosition());
                    telemetry.addData("Wobble Arm Speed", wobbleSystem.wobbleArm.get());
                    telemetry.addData("Wobble Hand Pos", wobbleSystem.wobbleHand.getPosition());
                    telemetry.update();
                    wobbleSystem.arm_down();
                } else if(elapsedTime.seconds() < 5.5) {
                    wobbleSystem.hand_open();
                } else {
                    requestOpModeStop();
                }
                break;
            case 1:
                break;
            case 4:
                break;
            default:
                stop();
        }
    }
}