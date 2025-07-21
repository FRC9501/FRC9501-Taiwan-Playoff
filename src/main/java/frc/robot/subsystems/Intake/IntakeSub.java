package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
    // private final SparkMax motor2 = new SparkMax(16, MotorType.kBrushless);
    private static final Distance LEdSpacing = Meters.of(1 / 120.0);
    private AnalogInput analog = new AnalogInput(0);
    private SparkMax motor;
    private AddressableLED led = new AddressableLED(9);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(250);  // LED 數量
    private AddressableLEDBufferView[] sectionViews;
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig motor2cfg = new SparkMaxConfig();
    private double flowOffset = 0; // 用於控制顏色流動的偏移量

    // 每個區域的 LED 數量
    private int[] sectionLengths = {15, 44, 41, 47, 51};

    public IntakeSub() {
        intakeConfig.inverted(true).idleMode(IdleMode.kBrake);
        led.setLength(ledBuffer.getLength());
        led.start();
        motor = new SparkMax(15, MotorType.kBrushless);
        motor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // motor2.configure(motor2cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 創建 LED 的不同區段
        sectionViews = new AddressableLEDBufferView[sectionLengths.length];
        int startIndex = 0;

        for (int i = 0; i < sectionLengths.length; i++) {
            int endIndex = startIndex + sectionLengths[i] - 1;
            sectionViews[i] = ledBuffer.createView(startIndex, endIndex);

            // 反轉分區2和分區4的方向
            if (i == 1 || i == 3) {
                sectionViews[i] = sectionViews[i].reversed();
            }

            startIndex = endIndex + 1;
        }
    }

    public double Distance() {
        return analog.getValue();
    }

    public void motorSet(double speed) {
        motor.set(speed);
        if (Distance() > 650) {
            motor.set(0);
        } else {
            motor.set(speed);
        }

        // motor2.set(speed2);
    }

    // public void motor2(double speed) {
    //     motor2.set(speed);
    // }

    public void shoot(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void LED() {
        // 當距離大於1000時，將LED顏色設為綠色
        if (Distance() > 1000) {
            greenLED();  // 將LED顯示為綠色
        } else {
            spaceFlowEffect();  // 顯示太空色系的流動效果
        }

        // 設定 LED 顯示
        led.setData(ledBuffer);
    }

    // 設定所有LED為綠色
    private void greenLED() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            // 設定所有LED為綠色（RGB = 0, 255, 0）
            ledBuffer.setRGB(i, 0, 255, 0);  
        }
    }

    // 太空顏色流動效果，使用藍色和紫色
    private void spaceFlowEffect() {
        flowOffset += 0.05;  // 控制流動速度，這個值可以調整流動的快慢

        // 燈條分為 5 個區域，每個區域的 LED 顆數在 sectionLengths 中
        for (int section = 0; section < sectionViews.length; section++) {
            AddressableLEDBufferView currentView = sectionViews[section];

            // 遍歷每個區域的 LED
            for (int i = 0; i < currentView.getLength(); i++) {
                double time = flowOffset + i * 0.2;  // 讓每個 LED 有不同的流動時間

                // 計算顏色，使用藍色和紫色的過渡
                int blueValue = (int) (Math.sin(time) * 128 + 127);  // 藍色
                int purpleValue = (int) (Math.sin(time + Math.PI / 2) * 128 + 127);  // 紫色

                // 設定顏色：藍色和紫色交替
                currentView.setRGB(i, purpleValue, 0, blueValue);  // 只有藍色和紫色，綠色為 0
            }
        }
    }

    @Override
    public void periodic() {
        LED();
        SmartDashboard.putNumber("Distance", Distance());
    }
}
