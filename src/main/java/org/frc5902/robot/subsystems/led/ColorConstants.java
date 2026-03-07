package org.frc5902.robot.subsystems.led;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.ToString;

public class ColorConstants {

    public static Color SOLID_RED =
            Color.builder().name("SOLID_RED").input(0.59).priority(0).build();
    public static Color SOLID_BLUE =
            Color.builder().name("SOLID_BLUE").input(0.85).priority(0).build();
    public static Color NO_COLOR = Color.builder().name("NO COLOR").build();

    @AllArgsConstructor
    @ToString
    @Builder
    public static class Color {
        @Builder.Default
        @Getter
        private double input = 0.99;

        @Builder.Default
        @Getter
        private String name = "";

        @Builder.Default
        @Getter
        private int priority = -1;
    }
}
