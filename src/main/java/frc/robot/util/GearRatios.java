package frc.robot.util;

public final class GearRatios {
    /**
     * UltraPlanetary doesn't have exact gear ratios, so this enum is a convenient way to record reductions.
     */
    public static enum UltraPlanetaryRatio {
        THREE_TO_ONE(84. / 29.), FOUR_TO_ONE(76. / 21.), FIVE_TO_ONE(68. / 13.);

        public final double ratio;

        UltraPlanetaryRatio(double ratio) {
            this.ratio = ratio;
        }
    }
}
