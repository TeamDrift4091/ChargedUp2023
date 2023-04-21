// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.team1891.common.LazyDashboard;

public class LEDString {
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private final int length;
    
    public LEDString(int port, int length) {
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        leds.setLength(buffer.getLength());
        this.length = length;

        LazyDashboard.addNumber("Estimated LED power draw", () -> {
            double totalPower = 0;
            for (int i = 0; i < length; i++) {
                Color color = buffer.getLED(i); 
                double sum = color.red + color.green + color.blue;
                totalPower += sum / 3.;
            }
            
            totalPower *= .06; // 60mA per led
            return totalPower;
        });
    }


    public void start() {
        leds.start();
    }

    public void stop() {
        leds.stop();
    }

    public void updateLEDs() {
        leds.setData(buffer);
    }

    public void individualPixel(int index, int hue) {
        individualPixel(index, hue, false);
    }
    
    public void individualPixel(int index, int hue, boolean clearOthers) {
        if (clearOthers) {
            for (int i = 0; i < length; i++) {
                if (i == index) {
                    buffer.setHSV(i, hue, 255, 128);
                } else {
                    buffer.setRGB(i, 0, 0, 0);
                }
            }    
        } else {
            buffer.setHSV(index, hue, 255, 128);
        }
    }

    public void individualPixel(int index, int r, int g, int b) {
        individualPixel(index, r, g, b, false);
    }
    
    public void individualPixel(int index, int r, int g, int b, boolean clearOthers) {
        if (clearOthers) {
            for (int i = 0; i < length; i++) {
                if (i == index) {
                    buffer.setRGB(i, r, g, b);
                } else {
                    buffer.setRGB(i, 0, 0, 0);
                }
            }    
        } else {
            buffer.setRGB(index, r, g, b);
        }
    }

    public void allOneColor(int hue) {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
            // Set the value
            buffer.setHSV(i, hue, 255, 128);
        }
    }

    public void allOneColor(int r, int g, int b) {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
            // Set the value
            buffer.setRGB(i, r, g, b);
        }
    }

    public void off() {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); ++i) {
            // Set the value
            buffer.setRGB(i, 0, 0, 0);
        }
    }

    private int rainbowFirstPixelHue = 0;
    public void rainbow() {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
          // Set the value
          buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue ++;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    public void fastRainbow() {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (rainbowFirstPixelHue + (2 * i * 180 / buffer.getLength())) % 180;
          // Set the value
          buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    public void flash(double intervalSeconds, Runnable runnable) {
        alternate(intervalSeconds, runnable, () -> {
            off();
            updateLEDs();
        });
    }
    
    public void alternate(double intervalSeconds, Runnable a, Runnable b) {
        long currentTime = System.currentTimeMillis();
        if ((currentTime % (int) (intervalSeconds * 2000)) < (int) (intervalSeconds * 1000)) {
            a.run();
        } else {
            b.run();
        }
    }
}
