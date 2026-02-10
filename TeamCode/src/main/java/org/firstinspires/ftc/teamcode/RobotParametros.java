package org.firstinspires.ftc.teamcode;

public class RobotParametros {
    public final double radPerTick;
    public final double wheelRadius;
    public final double Lr;

    public RobotParametros(double ticksPerRev, double reduction,
                           double wheelRadius, double Lr) {

        this.radPerTick = (2.0 * Math.PI) / (ticksPerRev * reduction);
        this.wheelRadius = wheelRadius;
        this.Lr = Lr;
    }
}
