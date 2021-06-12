package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot.DriveTrain;

@TeleOp(name = "MotorTest")
public class MotorTest extends OpMode {
    private DcMotor motor;
    private DriveTrain driveTrain;
    private ElapsedTime elapsedTime;
    boolean reset;
    double spinTime = 0;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("fw");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        elapsedTime = new ElapsedTime();
        reset = true;
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.left_stick_y);

        telemetry.addData("Flywheel Power", motor.getPower());
        telemetry.addData("Elapsed Time", spinTime);
    }

}
