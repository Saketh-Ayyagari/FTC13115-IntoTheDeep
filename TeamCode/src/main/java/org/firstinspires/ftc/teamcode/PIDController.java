package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Saketh Ayyagari
 * PIDController Class.
 * Given coefficients Kp, Ki, and Kd, maintain a specific setpoint
 */
public class PIDController{
    private double setpoint;
    private double Kp, Ki, Kd;
    private ElapsedTime runtime;
    private double error_sum;
    private double prev_error;
    public PIDController(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.runtime = new ElapsedTime();
        this.error_sum = 0;
        this.prev_error = 0;
    }
    public PIDController(double Kp){
        this.Kp = Kp;
        this.Ki = 0;
        this.Kd = 0;
        this.runtime = new ElapsedTime();
        this.error_sum = 0;
        this.prev_error = 0;
    }
    public double update(double setpoint, double actual){
        // calculate proportional term
        double error = setpoint - actual;
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
}
