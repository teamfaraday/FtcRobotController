package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robot.DriveTrain;
import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot.Intake;
import org.firstinspires.ftc.robot.WobbleSystem;
import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot.Hitter;
import org.firstinspires.ftc.robot_utilities.RotationController;
import org.firstinspires.ftc.robot_utilities.Vals;

@TeleOp(name = "EchoOp")
public class EchoOp extends OpMode {
    private FtcDashboard dashboard;
    private GamePadController gamepad;

    private DriveTrain driveTrain;
    private FlyWheel flywheel;
    private WobbleSystem wobbleSystem;
    private Hitter hitter;
    private Intake intake;

    private double intakeSpeed = 0;
    private boolean wobbleHandOpen = true;
    private WobbleArmState wobbleArmState = WobbleArmState.UP;
    private boolean flywheelOn = false;
    private boolean flywheelPowershot = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        gamepad = new GamePadController(gamepad1);

        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"), new Motor(hardwareMap, "dr"));
        driveTrain.resetEncoders();
        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE), telemetry);
        wobbleSystem = new WobbleSystem(new Motor(hardwareMap, "wobbleArmMotor"), hardwareMap.servo.get("wobbleArmServo"));
        wobbleSystem.hand_open();
        wobbleSystem.arm_up();
        hitter = new Hitter(hardwareMap.servo.get("sv"));
        intake = new Intake(new Motor(hardwareMap, "in1"),  new Motor(hardwareMap, "in2"));
    }

    @Override
    public void loop() {
        gamepad.update();

        double leftSpeed = gamepad1.left_stick_y;
        double rightSpeed = gamepad1.right_stick_y;
        if(gamepad1.right_trigger >= 0.1) {
            intake.intake_in();
        } else if(gamepad1.left_trigger >= 0.1) {
            intake.intake_out();
        } else {
            intake.intake_stop();
        }

        if(gamepad.isARelease()) {
            wobbleHandOpen = !wobbleHandOpen;
        }

        if(wobbleHandOpen) {
            wobbleSystem.hand_open();
        } else {
            wobbleSystem.hand_close();
        }

        if(gamepad.isUpRelease()) {
            wobbleArmState = WobbleArmState.UP;
        } else if(gamepad.isDownRelease()) {
            wobbleArmState = WobbleArmState.DOWN;
        } else if(gamepad.isLeftRelease()) {
            wobbleArmState = WobbleArmState.MID;
        }


        if(gamepad.isXRelease()) {
            flywheelPowershot = !flywheelPowershot;
        }
        if(gamepad.isYRelease()) {
            flywheel.flipDirection();
        }

        if(gamepad.isBRelease()) {
            if(flywheel.isOn()) {
                flywheelOn = false;
            } else {
                flywheelOn = true;
            }
        }

        if(flywheelOn) {
            if(flywheelPowershot) {
                flywheel.on_slow();
            } else {
                flywheel.on();
            }
        } else {
            flywheel.off();
        }

        String isReady = "FLYWHEEL NOT READY";

        if(flywheel.isReady()) {
            isReady = "FLYWHEEL READY!";
        }


        if(gamepad1.left_bumper) {
            hitter.hit();
        }
        else {
            hitter.reset();
        }

        driveTrain.setSpeed(leftSpeed, rightSpeed);

        switch (wobbleArmState) {
            case UP:
                wobbleSystem.arm_up();
                break;
            case MID:
                wobbleSystem.arm_mid();
                break;
            case DOWN:
                wobbleSystem.arm_down();
                break;
        }


        telemetry.addData("Flywheel Speed", flywheel.flywheel.get());
        telemetry.addData("Flywheel Velocity", flywheel.flywheel.getCorrectedVelocity());
        telemetry.addData("Flywheel Filtered Speed", Vals.flywheel_filtered_speed);
        telemetry.addData("Flywheel Position", flywheel.flywheel.getCurrentPosition());
        telemetry.addData("Hitter Position", hitter.hitter.getPosition());
        telemetry.addData("Left Speed", leftSpeed);
        telemetry.addData("Right Speed", rightSpeed);
        telemetry.addData("Intake Speed", intakeSpeed);
        telemetry.addData("Wobble Arm Position", wobbleSystem.wobbleArm.getCurrentPosition());
        telemetry.addData("Wobble Arm Rotations", wobbleSystem.wobbleArm.getCurrentPosition());
        telemetry.addData("Wobble Arm Distance", wobbleSystem.wobbleArm.getDistance());
        telemetry.addData("Wobble Hand Position", wobbleSystem.wobbleHand.getPosition());


    }
}

enum WobbleArmState {
    DOWN,
    MID,
    UP
}