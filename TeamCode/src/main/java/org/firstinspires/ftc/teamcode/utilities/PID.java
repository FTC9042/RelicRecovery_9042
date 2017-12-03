package org.firstinspires.ftc.teamcode.utilities;

/**
 * Created by anikaitsingh on 12/1/17.
 */

public class PID {

    public final double P_CONSTANT;
    public final double I_CONSTANT;
    public final double D_CONSTANT;

    public double target;
    public double err;
    public double prevError;
    public double cummErr;

    /** Full Fledged PID
     *
     * @param P proportional constant
     * @param I integral constant
     * @param D derivative constant
     */
    public PID(double P, double I, double D){
        this.P_CONSTANT = P;
        this.I_CONSTANT = I;
        this.D_CONSTANT = D;

        err = 0;
        prevError = 0;
        cummErr = 0;
    }

    /** P only control loop
     * @param P value of constant
     */
    public PID(double P){
        this(P, 0,0);
    }


    public void setTarget(double t){
        target = t;
    }

    public double getValue(double curr, double dt){
        err = target - curr;
        cummErr += err;

        double p = P_CONSTANT * err;
        double i = I_CONSTANT * cummErr;
        double d = D_CONSTANT * (err - prevError)/dt;

        prevError = err;

        return p + i + d;
    }

    public double getValue(double target, double curr, double dt){
        setTarget(target);
        return getValue(curr, dt);
    }


}
