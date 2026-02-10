package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class AutonomousMode extends OpMode {
    //Reservas y definiciones
    private DcMotorEx motorLeft = null;
    private DcMotorEx motorRight = null;
    private RobotParametros params = null;
    State stateRobot = State.IDLE;


    //Variables de accion
    double targetAngleDeg = 0;
    int turnDirection = 1;

    double targetDistance = 0;

    double targetTime = 0;

    @Override
    public void init() {
        telemetry.addData("Info", "sistema de control autónomo");
        telemetry.addData("Init", "Configurando.");

        motorLeft = hardwareMap.get(DcMotorEx.class, "motor_00");
        motorRight = hardwareMap.get(DcMotorEx.class, "motor_01");

        motorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setTargetPosition(0);
        motorRight.setTargetPosition(0);

        motorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        motorLeft.setPower(0.3);
        motorRight.setPower(0.3);

        telemetry.addData("Motores", "configurados.");

        params = new RobotParametros(28, 20, 6, 13);
        telemetry.addData("Parametros de robot", "cargados");

        telemetry.addData("Init", "Configurado.");
        telemetry.addData("FSM", stateRobot);
    }

    @Override
    public void loop() {

        switch (stateRobot) {

            case IDLE:
                commandTurn(-1, 90);
                break;

            case TURNING:
                if (!motorLeft.isBusy() && !motorRight.isBusy()) {
                    commandMove(2000);
                }
                break;

            case MOVING:
                if (!motorLeft.isBusy() && !motorRight.isBusy()) {
                    commandWait(2000);
                }
                break;

            case SAFE:
                // futuro: temporizador
                break;
        }

        telemetry.addData("Estado", stateRobot);
        telemetry.update();
    }


    public void setState(State newState){
        stateRobot = newState;

        switch(stateRobot){
            case IDLE:
                motorRight.setPower(0);
                motorLeft.setPower(0);
                break;
            case TURNING:
                turnRobot(turnDirection, targetAngleDeg);
                runRobot();
                powerRobot();
                break;
            case MOVING:
                moveRobot((int)targetDistance /*falta implementar (ticks actualmente)*/);
                runRobot();
                powerRobot();
                break;
            case SAFE:
                //esperar a que la persona ponga la pelota
                motorRight.setPower(0);
                motorLeft.setPower(0);
                break;
        }
    }

    public void commandWait(double tiempo){
        targetTime = tiempo;
        setState(State.SAFE);
    }

    public void commandTurn(int direction, double angleDeg) {
        turnDirection = direction;
        targetAngleDeg = angleDeg;
        setState(State.TURNING);
    }

    public void commandMove(double distance) {
        targetDistance = distance;
        setState(State.MOVING);
    }

    public void runRobot(){
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void powerRobot(){
        motorRight.setPower(0.3);
        motorLeft.setPower(0.3);
    }

    //Por ahora solo acepta ticks, pero debemos tener medidas de cinematica
    public void moveRobot(int targetTicks){
        motorLeft.setTargetPosition(targetTicks);
        motorRight.setTargetPosition(targetTicks);
    }

    public void turnRobot(int sentido_rotacion, double angleDeg) {
        double angle = Math.toRadians(angleDeg);
        double s = params.Lr * angle; // arco de cada rueda
        double ticksDouble = s / (params.wheelRadius * params.radPerTick);

        int ticks_x_rotacion = (int) ticksDouble;

        motorLeft.setTargetPosition(sentido_rotacion * ticks_x_rotacion);
        motorRight.setTargetPosition(-sentido_rotacion * ticks_x_rotacion);
    }
}

enum State {
    IDLE, //Esperando a comenzar
    TURNING, // Girando
    MOVING, // MOviendose
    SAFE // esperando la recarga

}