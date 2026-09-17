package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;

/**
 * Tester standalone do SHOOTER — bancada, sem depender do {@code Robot}/drivetrain, e sem PIDF
 * (power direto nos motores, não RPM fechado — é só pra checar fiação/direção/força bruta).
 *
 * <ul>
 *   <li><b>Right Trigger</b>: power dos dois flywheels ({@code rightShooterMotor} +
 *       {@code leftShooterMotor}), juntos, mesma potência.</li>
 * </ul>
 *
 * @author LucasDiegoHD - Team #23069
 */
@TeleOp(name = "TESTER - Shooter", group = "Testers")
public class ShooterTester extends OpMode {

    private DcMotorEx ShooterMotor;

    @Override
    public void init() {
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        ShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        double power = gamepad1.right_trigger;
        double reversePower = -gamepad1.left_trigger;

        if (gamepad1.right_trigger > 0.1){
            ShooterMotor.setPower(power);
        }
       else if (gamepad1.left_trigger > 0.1){
            ShooterMotor.setPower(reversePower);
        }
        else {
            ShooterMotor.setPower(0);
        }

        telemetry.addData("Power (comandado)", "%.2f", ShooterMotor.getPower());
        telemetry.update();
    }
}
