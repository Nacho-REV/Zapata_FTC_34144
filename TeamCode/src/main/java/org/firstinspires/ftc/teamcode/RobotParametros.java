package org.firstinspires.ftc.teamcode;


public class RobotParametros {
    public final double TICKS_PER_RAD;
    public final double GEAR_REDUCTION;
    public final double DIST_WHEEL_2_CIR; //mm

    public final double WHEEL_RADIUS; //mm


    public RobotParametros(double TICKS_PER_RAD, double GEAR_REDUCTION,
                           double WHEEL_RADIUS, double DIST_WHEEL_2_CIR) {

        this.TICKS_PER_RAD = TICKS_PER_RAD / (2*Math.PI);
        this.WHEEL_RADIUS = WHEEL_RADIUS;
        this.DIST_WHEEL_2_CIR = DIST_WHEEL_2_CIR;
        this.GEAR_REDUCTION = GEAR_REDUCTION;
    }
}
