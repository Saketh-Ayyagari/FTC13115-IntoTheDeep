package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Saketh Ayyagari:
 * PIDController Class.
 * Given coefficients Kp, Ki, and Kd, return the output to move to a setpoint
 */
public class PIDController{
    private double setpoint;
    private double Kp, Ki, Kd;
    private ElapsedTime runtime;
    private double error_sum;
    private double prev_error;
    // should the error be angle-wrapped? Only used for gyro turning/dealing with angles
    // inspired by website CTRL-ALT-FTC
    private boolean isAngleWrapped;
    // full PID Controller
    public PIDController(double Kp, double Ki, double Kd, boolean isAngleWrapped){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.runtime = new ElapsedTime();
        this.error_sum = 0;
        this.prev_error = 0;
        this.isAngleWrapped = isAngleWrapped;
    }
    public PIDController(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.runtime = new ElapsedTime();
        this.error_sum = 0;
        this.prev_error = 0;
        this.isAngleWrapped = false;
    }
    // proportional controller
    public PIDController(double Kp, boolean isAngleWrapped){
        this.Kp = Kp;
        this.Ki = 0;
        this.Kd = 0;
        this.runtime = new ElapsedTime();
        this.error_sum = 0;
        this.prev_error = 0;
        this.isAngleWrapped = isAngleWrapped;
    }
    public PIDController(double Kp){
        this.Kp = Kp;
        this.Ki = 0;
        this.Kd = 0;
        this.runtime = new ElapsedTime();
        this.error_sum = 0;
        this.prev_error = 0;
        this.isAngleWrapped = false;
    }
    // returns update to move to setpoint given both setpoint and actual value
    public double update(double setpoint, double actual){
        // calculate proportional term
        double error;
        if (isAngleWrapped){
            error = angleWrap(setpoint - actual);
        }
        else{
            error = setpoint - actual;
        }
        double P = this.Kp*error;
        // calculate integral term
        error_sum += error * runtime.seconds();
        double I = Ki*error_sum;
        // calculate derivative term
        double derivative = (error - prev_error)/runtime.seconds();
        prev_error = error;
        double D = Kd*derivative;
        // reset runtime for further calculation
        runtime.reset();

        return P + I + D;
    }
    /**
     * This function normalizes the angle so it returns a value between
     * -180° and 180° instead of 0° to 360°.
     * Used for wrapping angle error to ensure robot moves in the shortest possible distance & doesn't spin infinitely.
     * INSPIRED BY CTRL-ALT-FTC
     **/
    private double angleWrap(double degrees) {
        if (degrees < 360 && degrees > 180) {
            degrees -= 360;
        }
        // keep in mind that the result is in radians
        return degrees;
    }
}
