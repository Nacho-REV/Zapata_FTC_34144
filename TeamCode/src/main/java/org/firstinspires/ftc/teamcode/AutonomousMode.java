package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous
public class AutonomousMode extends LinearOpMode {

    private static final boolean FLAG_VIEW_DATA_RUNNING = true;
    private static final double CURRENT_VOID_LOAD_MOTOR_MILIAMPS = 50;
    private DcMotorEx motorRight = null;
    private DcMotorEx motorLeft = null;

    RobotParametros parametros = null;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode(){
        telemetry.addLine("Modo de operacion autonomo");
        telemetry.addLine(" ");


        motorRight = hardwareMap.get(DcMotorEx.class, "motor_00");
        motorLeft = hardwareMap.get(DcMotorEx.class, "motor_01");

        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        motorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorRight.setTargetPosition(0);
        motorLeft.setTargetPosition(0);

        motorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        telemetry.addData("Motores de tren motriz", "configurados");

        parametros = new RobotParametros(28, 20, 6, 6);

        telemetry.addData("Parámetros de Robot", "cargados");


        /*el robot se mueve para detectar corriente de umbral en entorno*/
        telemetry.addData("Calibración de umbral para DOSC", "calibrado");
        isMotorConnected(false);
        telemetry.update();

        waitForStart();

        turnRobot(90, 0.5, 6);
        turnRobot(-90, 0.5, 6);

    }
    //public void driveForward(){}

    public void turnRobot(double angleDeg, double speed, double timeoutS){
        /*
        * metodo que conoce el robot
        * */
        double angleRad = Math.toRadians(angleDeg);

        int ticks;
        double arcoRobot = angleRad * parametros.DIST_WHEEL_2_CIR;
        double angleWheel = arcoRobot / parametros.WHEEL_RADIUS;
        double angleShaftMotor = parametros.GEAR_REDUCTION * angleWheel;
        ticks = (int) (angleShaftMotor * parametros.TICKS_PER_RAD);

        encoderDrive(speed, ticks, -ticks, timeoutS);

    }

    public void encoderDrive(double speed,
                             int leftTicks, int rightTicks,
                             double timeoutS) {

        int newRightTarget;
        int newLeftTarget;

        if(opModeIsActive()) {
            newRightTarget = motorRight.getCurrentPosition() + rightTicks;
            newLeftTarget = motorLeft.getCurrentPosition() + leftTicks;

            motorRight.setTargetPosition(newRightTarget);
            motorLeft.setTargetPosition(newLeftTarget);

            motorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motorRight.setPower(Math.abs(speed));
            motorLeft.setPower(Math.abs(speed));

            while(opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorRight.isBusy() && motorLeft.isBusy())){
                if(FLAG_VIEW_DATA_RUNNING){
                    telemetry.addLine("==CORRIENTE==");
                    telemetry.addData("Tren derecho", motorRight.getCurrent(CurrentUnit.MILLIAMPS));
                    telemetry.addData("Tren izquierdo", motorLeft.getCurrent(CurrentUnit.MILLIAMPS));

                    telemetry.addLine("==TICKS==");
                    telemetry.addData("Tren derecho", newRightTarget - motorRight.getCurrentPosition());
                    telemetry.addData("Tren izquierdo", newLeftTarget - motorLeft.getCurrentPosition());

                    isMotorConnected(true);
                    telemetry.update();
                    idle();
                }
            }

            motorRight.setPower(0);
            motorLeft.setPower(0);

            motorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public void isMotorConnected(boolean inTask) {
        if(!inTask){
            motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motorRight.setPower(0.5);
            motorLeft.setPower(0.5);

            sleep(200);
        }


        double currentRight = motorRight.getCurrent(CurrentUnit.MILLIAMPS);
        double currentLeft  = motorLeft.getCurrent(CurrentUnit.MILLIAMPS);

        //telemetry.addData("motorRight (mA)", currentRight);
        //telemetry.addData("motorLeft  (mA)", currentLeft);

        boolean isMotorRightConnected = currentRight > CURRENT_VOID_LOAD_MOTOR_MILIAMPS;
        boolean isMotorLeftConnected  = currentLeft  > CURRENT_VOID_LOAD_MOTOR_MILIAMPS;

        if(!isMotorRightConnected) telemetry.addLine("[WARNING] motor derecho("+motorRight.getDeviceName()+") no conectado");
        if(!isMotorLeftConnected)  telemetry.addLine("[WARNING] motor izquierdo("+motorLeft.getDeviceName()+") no conectado");

        if(!inTask){
            motorRight.setPower(0);
            motorLeft.setPower(0);
        }
    }


}