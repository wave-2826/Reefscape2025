package frc.robot.util.sim.simField;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.SimulatedArena.Simulatable;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class CustomReefSimulation implements Simulatable {
    private final SimulatedReefscapeArena arena;
    private final List<CustomReefBranchesTower> branchTowers;
    private final List<CustomCoralHolder> coralHolders;

    public CustomReefSimulation(SimulatedReefscapeArena arena) {
        this.arena = arena;
        this.coralHolders = new ArrayList<>(96);
        this.branchTowers = new ArrayList<>(24);

        Translation2d origin = new Translation2d(FieldMirroringUtils.FIELD_WIDTH / 2,
            FieldMirroringUtils.FIELD_HEIGHT / 2);
        Translation2d[] branchesCenterPositionBlue = new Translation2d[] {
            new Translation2d(-4.810, 0.164).plus(origin), // A
            new Translation2d(-4.810, -0.164).plus(origin), // B
            new Translation2d(-4.690, -0.373).plus(origin), // C
            new Translation2d(-4.406, -0.538).plus(origin), // D
            new Translation2d(-4.164, -0.537).plus(origin), // E
            new Translation2d(-3.879, -0.374).plus(origin), // F
            new Translation2d(-3.759, -0.164).plus(origin), // G
            new Translation2d(-3.759, 0.164).plus(origin), // H
            new Translation2d(-3.880, 0.373).plus(origin), // I
            new Translation2d(-4.164, 0.538).plus(origin), // J
            new Translation2d(-4.405, 0.538).plus(origin), // K
            new Translation2d(-4.690, 0.374).plus(origin) // L
        };
        Translation2d[] branchesCenterPositionRed = Arrays.stream(branchesCenterPositionBlue)
            .map(FieldMirroringUtils::flip).toArray(Translation2d[]::new);
        Rotation2d[] branchesFacingOutwardsBlue = new Rotation2d[] {
            Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180), // A and B
            Rotation2d.fromDegrees(-120), Rotation2d.fromDegrees(-120), // C and D
            Rotation2d.fromDegrees(-60), Rotation2d.fromDegrees(-60), // E and F
            Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), // G and H
            Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(60), // I and J
            Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(120), // K and L
        };
        Rotation2d[] branchesFacingOutwardsRed = Arrays.stream(branchesFacingOutwardsBlue)
            .map(FieldMirroringUtils::flip).toArray(Rotation2d[]::new);

        CustomReefBranchesTower branch;
        for(int i = 0; i < 12; i++) {
            // blue
            branch = new CustomReefBranchesTower(branchesCenterPositionBlue[i], branchesFacingOutwardsBlue[i]);
            branchTowers.add(branch);
            coralHolders.addAll(branch.coralHolders());

            // red
            branch = new CustomReefBranchesTower(branchesCenterPositionRed[i], branchesFacingOutwardsRed[i]);
            branchTowers.add(branch);
            coralHolders.addAll(branch.coralHolders());
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        Set<GamePieceProjectile> gamePiecesLaunched = arena.gamePieceLaunched();
        Set<GamePieceProjectile> toRemoves = new HashSet<>();
        for(CustomCoralHolder coralHolder : coralHolders)
            for(GamePieceProjectile gamePieceLaunched : gamePiecesLaunched)
                checkForCoralPlacement(coralHolder, gamePieceLaunched, toRemoves);

        for(GamePieceProjectile toRemove : toRemoves) gamePiecesLaunched.remove(toRemove);
    }

    private void checkForCoralPlacement(CustomCoralHolder coralHolder, GamePieceProjectile gamePieceLaunched,
        Set<GamePieceProjectile> toRemove) {
        // HACK, but whatever
        if(gamePieceLaunched instanceof ReefscapeCoralFromTransform coral) {
            if(!toRemove.contains(gamePieceLaunched) && coralHolder.checkCoralPlacement(coral.getPose3d(),
                coral.getVelocity3dMPS())) toRemove.add(gamePieceLaunched);
        }
        if(gamePieceLaunched instanceof ReefscapeCoralOnFly coral) {
            if(!toRemove.contains(gamePieceLaunched) && coralHolder.checkCoralPlacement(coral.getPose3d(),
                coral.getVelocity3dMPS())) toRemove.add(gamePieceLaunched);
        }
    }

    /**
     * Displays all the CORALs scored on the REEF.
     *
     * @param coralPosesToDisplay a list of {@link Pose3d} objects used to visualize the positions of the CORALs on
     *            AdvantageScope
     */
    public void addCoralsOnReefForDisplay(List<Pose3d> coralPosesToDisplay) {
        for(CustomCoralHolder coralHolder : coralHolders) coralHolder.addContainedCoralsForDisplay(coralPosesToDisplay);
    }

    /** Clears all the CORALs scored on the REEF. */
    public void clearReef() {
        for(CustomReefBranchesTower tower : branchTowers) tower.clearBranches();
    }

    /**
     * Obtains the amount of <strong>CORAL</strong> held on the <strong>BRANCHES</strong>.
     *
     * <p>
     * This method returns a 2D array of size 12 x 4, where each entry represents the number of <strong>CORAL</strong>s
     * held on a particular branch.
     *
     * <p>
     * The <strong>BRANCHES</strong> are tracked in FMS as A, B, C, D, E, F, G, H, I, J, K, L (as per the game manual),
     * and are mapped to indices 0, 1, 2, ... in the array.
     *
     * <p>
     * The [i][j] entry in the array represents the number of <strong>CORAL</strong>(s) held on the L<code>j-1</code>
     * branch in the <code>i</code>th section.
     *
     * <p>
     * For example, <code>getBranches()[2][3]</code> returns the number of CORALs held on L4 of Branch C.
     *
     * <p>
     * Note that L2, L3, and L4 can only hold one <strong>CORAL</strong>, while L1 can hold up to two
     * <strong>CORAL</strong>s.
     *
     * @param side the alliance side (Red or Blue) to check for CORAL counts
     * @return a 2D array where each entry represents the number of <strong>CORAL</strong> held on each branch
     */
    public int[][] getBranches(DriverStation.Alliance side) {
        int[][] coralsCountOnBranches = new int[12][4];
        for(int i = 0; i < 12; i++) {
            CustomReefBranchesTower tower = branchTowers.get(side == DriverStation.Alliance.Red ? i * 2 + 1 : i * 2);
            coralsCountOnBranches[i][0] = tower.L1.coralCount;
            coralsCountOnBranches[i][1] = tower.L2.hasCoral ? 1 : 0;
            coralsCountOnBranches[i][2] = tower.L3.hasCoral ? 1 : 0;
            coralsCountOnBranches[i][3] = tower.L4.hasCoral ? 1 : 0;
        }

        return coralsCountOnBranches;
    }

    /**
     * Returns an optional instance.
     *
     * @return (optionally) an instance of this class, empty if
     */
    public static Optional<CustomReefSimulation> getInstance() {
        if(SimulatedArena.getInstance() instanceof SimulatedReefscapeArena arena) return Optional
            .of(arena.reefSimulation);
        return Optional.empty();
    }
}

/**
 * <h1>Simulates a BRANCHES tower in the REEF.</h1>
 *
 * <p>
 * A BRANCHES TOWER is a purple tower containing:
 *
 * <ul>
 * <li>The L1 TROUGH.
 * <li>The L2, L3, and L4 BRANCHES.
 * </ul>
 */
class CustomReefBranchesTower {
    public final CustomReefTrough L1;
    public final CustomReefBranch L2, L3, L4;

    public CustomReefBranchesTower(Translation2d stickCenterPositionOnField, Rotation2d facingOutwards) {
        // L1 trough, 15cm away from center
        this.L1 = new CustomReefTrough(stickCenterPositionOnField.plus(new Translation2d(0.15, facingOutwards)),
            facingOutwards);

        // L2 stick, 20 cm away from center, 78cm above ground, 35 deg pitch
        this.L2 = new CustomReefBranch(stickCenterPositionOnField.plus(new Translation2d(0.2, facingOutwards)),
            facingOutwards, 0.77, Math.toRadians(-35));

        // L3 stick, 20 cm away from center, 118cm above ground, 35 deg pitch
        this.L3 = new CustomReefBranch(stickCenterPositionOnField.plus(new Translation2d(0.2, facingOutwards)),
            facingOutwards, 1.17, Math.toRadians(-35));

        // L4 stick, 30 cm away from center, 178cm above ground, vertical
        this.L4 = new CustomReefBranch(stickCenterPositionOnField.plus(new Translation2d(0.26, facingOutwards)),
            facingOutwards, 1.78, Math.toRadians(-90));
    }

    /** Clears all the CORALs on the BRANCHES */
    public void clearBranches() {
        L1.coralCount = 0;
        L2.hasCoral = L3.hasCoral = L4.hasCoral = false;
    }

    /** Obtains a collection of the {@link CustomCoralHolder}s that are in this BRANCHES tower. */
    public Collection<CustomCoralHolder> coralHolders() {
        return List.of(L1, L2, L3, L4);
    }
}

/**
 * <h2>Represents a BRANCH or a TROUGH.</h2>
 *
 * <p>
 * Represents a structure on which <strong>CORAL</strong>s can be scored.
 */
sealed interface CustomCoralHolder permits CustomReefBranch, CustomReefTrough {
    /**
     * Checks if a CORAL has been successfully scored on this holder.
     *
     * @param coralOnFly the {@link ReefscapeCoralInterface} to check for placement
     * @return true if the CORAL is successfully scored, false otherwise
     */
    boolean checkCoralPlacement(Pose3d pose, Translation3d velocityMPSy);

    /**
     * Displays the positions of the CORALs that are on this holder.
     *
     * @param coralPosesToDisplay a list of poses used to visualize CORALs on AdvantageScope
     */
    void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay);
}

/**
 * <h1>Simulates a BRANCH (L2, L3, or L4) on a {@link CustomReefBranchesTower}.</h1>
 *
 * <p>
 * If a <strong>CORAL</strong> ejected into the air is in contact with this <strong>BRANCH</strong> at the correct
 * angle, as detected by {@link #checkCoralPlacement(ReefscapeCoralInterface)}, it will get attached.
 */
final class CustomReefBranch implements CustomCoralHolder {
    private final Pose3d idealCoralPlacementPose;
    public boolean hasCoral;

    CustomReefBranch(Translation2d idealPlacementPosition, Rotation2d facingOutwards, double heightMeters,
        double branchInwardsDirectionPitchRad) {
        this.idealCoralPlacementPose = new Pose3d(idealPlacementPosition.getX(), idealPlacementPosition.getY(),
            heightMeters,
            new Rotation3d(0, -branchInwardsDirectionPitchRad, facingOutwards.plus(Rotation2d.k180deg).getRadians()));
        this.hasCoral = false;
    }

    @Override
    public boolean checkCoralPlacement(Pose3d pose, Translation3d velocityMPS) {
        Transform3d positionDifference = pose.minus(idealCoralPlacementPose);

        boolean goingDown = velocityMPS.getZ() <= 0;
        boolean positionCorrection = positionDifference.getTranslation().toTranslation2d().getNorm() < Units
            .inchesToMeters(5) && Math.abs(positionDifference.getZ()) < Units.inchesToMeters(2.5);

        boolean targetHit = positionCorrection && goingDown && (!this.hasCoral);
        if(!targetHit) return false;

        return this.hasCoral = true;
    }

    @Override
    public void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay) {
        if(hasCoral) coralPosesToDisplay.add(idealCoralPlacementPose);
    }
}

/**
 * <h1>Simulates a TROUGH (L1) on a {@link CustomReefBranchesTower}.</h1>
 *
 * <p>
 * If a <strong>CORAL</strong> ejected into the air is in contact with this <strong>TROUGH</strong> at the correct
 * angle, as detected by {@link #checkCoralPlacement(ReefscapeCoralInterface)}, it will get attached.
 */
final class CustomReefTrough implements CustomCoralHolder {
    private final Pose3d firstPlacementPose, secondPlacementPose;
    private final Translation3d idealPlacementPosition;
    public int coralCount;

    protected CustomReefTrough(Translation2d centerPosition, Rotation2d outwardsFacing) {
        Rotation3d coralRotation = new Rotation3d(0, 0, outwardsFacing.plus(Rotation2d.kCCW_90deg).getRadians());
        Translation2d firstPosition = centerPosition.plus(new Translation2d(0.08, outwardsFacing));
        Translation2d secondPosition = centerPosition.plus(new Translation2d(-0.04, outwardsFacing));
        this.firstPlacementPose = new Pose3d(firstPosition.getX(), firstPosition.getY(), 0.48, coralRotation);
        this.secondPlacementPose = new Pose3d(secondPosition.getX(), secondPosition.getY(), 0.52, coralRotation);
        this.idealPlacementPosition = new Translation3d(centerPosition.getX(), centerPosition.getY(), 0.47);
        this.coralCount = 0;
    }

    @Override
    public boolean checkCoralPlacement(Pose3d pose, Translation3d velocityMPS) {
        if(coralCount >= 2) return false;

        Translation3d difference = pose.getTranslation().minus(idealPlacementPosition);
        boolean closeEnough = difference.toTranslation2d().getNorm() < Units.inchesToMeters(10)
            && Math.abs(difference.getZ()) < Units.inchesToMeters(5);

        if(closeEnough) coralCount++;
        return closeEnough;
    }

    @Override
    public void addContainedCoralsForDisplay(List<Pose3d> coralPosesToDisplay) {
        if(coralCount > 0) coralPosesToDisplay.add(firstPlacementPose);
        if(coralCount > 1) coralPosesToDisplay.add(secondPlacementPose);
    }
}
