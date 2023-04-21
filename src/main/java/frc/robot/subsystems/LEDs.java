// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utility.LEDString;

@SuppressWarnings("unused")
public class LEDs extends SubsystemBase {
  private static LEDs instance = null;
  public static LEDs getInstance() {
    if (instance == null) {
      instance = new LEDs();
    }
    return instance;
  }

  public static final int LENGTH = 147;
  public enum LEDMode {
    OFF,
    DISCONNECTED,
    DISABLED,
    AUTONOMOUS,
    TELEOP,
    TELEOP_SPECIAL,
    FAULT,
    CUBE_TARGET,
    CUBE_HOLD;
  }
  private LEDMode currentMode = null; // Set to OFF on init

  private final LEDString leds;

  private final AtomicReference<Consumer<LEDString>> ledConsumer = new AtomicReference<Consumer<LEDString>>(null);
  private final Notifier periodicThread;

  private LEDs() {
    leds = new LEDString(9, LENGTH);

    periodicThread = new Notifier(() -> {
      Consumer<LEDString> consumer = ledConsumer.get();
      if (consumer != null) {
        consumer.accept(leds);
      }
    });
    periodicThread.setName("LED periodic");
    periodicThread.startPeriodic(.02);

    setMode(LEDMode.OFF);
  }

  public void start() {
    leds.start();
  }

  public void stop() {
    leds.stop();
  }

  public void setCustomConsumer(Consumer<LEDString> customConsumer) {
    ledConsumer.set(customConsumer);
    currentMode = null;
  }

  public void setMode(LEDMode mode) {
    if (mode != currentMode) {
      currentMode = mode;
      switch (currentMode) {
        case OFF:
          ledConsumer.set((leds) -> off(leds));
          break;
        case DISCONNECTED:
          ledConsumer.set((leds) -> disconnected(leds));
          break;
        case DISABLED:
          ledConsumer.set(
            (leds) -> setOnce(() -> setAll(leds, 0, 50, 0)));
          break;
        case AUTONOMOUS:
          ledConsumer.set((leds) -> rainbow(leds));
          break;
        case TELEOP:
          // Show alliance color
          if (Robot.isBlueAlliance()) {
            ledConsumer.set((leds) -> setOnce(() -> twoColor(leds, 2, 0, 0, 200, 150, 150, 150)));
          } else {
            ledConsumer.set((leds) -> setOnce(() -> twoColor(leds, 2, 200, 0, 0, 150, 150, 150)));
          }
          break;
        case TELEOP_SPECIAL:
          // Show alliance color
          if (Robot.isBlueAlliance()) {
            ledConsumer.set((leds) -> leds.alternate(.5, 
              () -> twoColor(leds, 4, 0, 0, 200, 150, 150, 150),
              () -> twoColor(leds, 4, 150, 150, 150, 0, 0, 200)));
          } else {
            ledConsumer.set((leds) -> leds.alternate(.5, 
              () -> twoColor(leds, 4, 200, 0, 0, 150, 150, 150),  
              () -> twoColor(leds, 4, 150, 150, 150, 200, 0, 0)));
          }
          break;
        case FAULT:
          ledConsumer.set((leds) -> leds.alternate(.25,
            () -> setAll(leds, 255, 0, 0), 
            () -> setAll(leds, 100, 0, 0)
          ));
          break;
        case CUBE_TARGET:
          ledConsumer.set((leds) -> fastRainbow(leds));
          break;
        case CUBE_HOLD:
          ledConsumer.set((leds) -> {
            setOnce(() -> setAll(leds, 40, 20, 80));
          });
          break;
      }
    }
  }

  private void setAll(LEDString leds, int r, int g, int b) {
    leds.allOneColor(r, g, b);
    leds.updateLEDs();
  }

  private void setOnce(Runnable runnable) {
    runnable.run();
    ledConsumer.set(null);
  }

  private void off(LEDString leds) {
    leds.off();
    leds.updateLEDs();
    ledConsumer.set(null);
  }

  private int i = 0;
  private boolean isIncreasing = true;
  private void disconnected(LEDString leds) {
    if (i + 4 > LENGTH) {
      isIncreasing = false;
    }
    if (i == 0) {
      isIncreasing = true;
    }
    i += isIncreasing ? 1 : -1;
    leds.individualPixel((i) % LENGTH, 150, 150, 150, true);
    leds.individualPixel((i + 1) % LENGTH, 150, 150, 150);
    leds.individualPixel((i + 2) % LENGTH, 150, 150, 150);
    leds.individualPixel((i + 3) % LENGTH, 150, 150, 150);
    leds.updateLEDs();
  }

  private void rainbow(LEDString leds) {
    leds.rainbow();
    leds.updateLEDs();
  }

  private void fastRainbow(LEDString leds) {
    leds.fastRainbow();
    leds.updateLEDs();
  }

  private void twoColor(LEDString leds, int spacing, int r1, int g1, int b1, int r2, int g2, int b2) {
    for (int i = 0; i < LENGTH; i++) {
      if (i / spacing % 2 == 0) {
        leds.individualPixel(i, r1, g1, b1);
      } else {
        leds.individualPixel(i, r2, g2, b2);
      }
    }
    leds.updateLEDs();
  }


  @Override
  public void periodic() {}
}
