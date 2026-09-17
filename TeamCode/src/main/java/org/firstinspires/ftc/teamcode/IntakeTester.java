package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;

/**
 * Tester standalone do INTAKE — bancada, sem depender do {@code Robot}/drivetrain.
 *
 * <ul>
 *   <li><b>Right Trigger</b>: power do {@code intakeMotor} (0 a 1).</li>
 *   <li><b>Left Trigger</b>: power do {@code triggerMotor} / gatilho-alimentador (0 a 1).</li>
 * </ul>
 *
 * @author LucasDiegoHD - Team #23069
 */
@TeleOp(name = "TESTER - Intake", group = "Testers")
public class IntakeTester extends OpMode {

    private DcMotorEx intakeMotor;

    private Servo servo;
    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.INTAKE_MOTOR);
        servo = hardwareMap.get(Servo.class, "Servo");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double intakePower = gamepad1.right_trigger;
        double reverseIntakeMotor = -gamepad1.left_trigger;

        if (gamepad1.right_trigger > 0.1){
            intakeMotor.setPower(intakePower);
            servo.setPosition(1.0);
        }
        else if (gamepad1.left_trigger > 0.1){
            intakeMotor.setPower(reverseIntakeMotor);
        }
        else {
            intakeMotor.setPower(0);
            servo.setPosition(0.0);
        }

        telemetry.addData("intakeMotor power", "%.2f", intakeMotor.getPower());
        telemetry.update();
    }
}
