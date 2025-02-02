package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;

public class IntakeIOReal implements IntakeIO {
    SparkMax frontBottomMotor;
    SparkMax frontTopMotor;

    SparkMax agitatorMotor1;
    SparkMax agitatorMotor2;

    SparkMax beltMotor;
}
