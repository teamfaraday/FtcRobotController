package org.firstinspires.ftc.robot_utilities;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class FlyWheel {

    public Motor flywheel;

    private double flywheelSpeed = 0;
    private double flywheelDirection = Vals.flywheel_direction;

    public FlyWheel(Motor flywheel) {
        this.flywheel = flywheel;

        this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheel.setVeloCoefficients(Vals.flywheel_kp, Vals.flywheel_ki, Vals.flywheel_kd);
        this.flywheel.setFeedforwardCoefficients(Vals.flywheel_ks, Vals.flywheel_kv);
    }

    private void set() {
        this.flywheel.set(this.flywheelDirection * this.flywheelSpeed);
    }

    public void on() {
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheelSpeed = Vals.flywheel_speed;

        this.set();
    }

    public void off() {
        flywheel.setRunMode(Motor.RunMode.RawPower);
        flywheelSpeed = 0;

        this.set();
    }

    public boolean isOn() {
        return flywheelSpeed != 0;
    }

    public void flipDirection() {
        this.flywheelDirection *= -1;

        this.set();
    }
}