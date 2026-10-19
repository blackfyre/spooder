# Design

## Context

`gait::footTarget` computes two paths per leg for the current cycle position: a straight path (the translate vector `v`, rotated into the leg frame by the leg's placement angle) and a rotation path (a stroke of `rotate × rotationScale` along the leg frame's Y axis, with an outward bulge at mid-stroke). It returns their blend `(straight × |f| + rotation × |t|) / (|f| + |t|)`, where `f` is the translate magnitude and `t` the rotate amount. Because the weights sum to one, each motion is scaled by its share of the total, which is what costs the forward stride when turning. Progress through the cycle already follows `max(f, |t|)`, so the cadence is right; only the stroke geometry needs to change.

## Goals / Non-Goals

**Goals:**

- Forward walking plus turning gives an arc at the forward stride the translate input alone would give, until a stride limit is reached.
- Unchanged paths for translate-only and rotate-only input.
- Car-like steering from one stick, with the crab layout kept.

**Non-Goals:**

- Exact rigid-body kinematics from measured leg mounts. The rotation stroke keeps its leg-frame approximation; the body geometry from the STLs is a later change (it is also what `add-gestures` needs for body sway).
- New gaits, or changes to cadence and lift.

## Decisions

### D1. Superpose displacements around the stance point

Each path is turned into a displacement from the leg's stance point `S = (STANCE_DISTANCE, 0, ground)`: `dS = straight − S`, `dR = rotation − S`. The foot target is `S + k·(wS·dS_xy + wR·dR_xy)` in X and Y, with the full weights `wS = wR = 1` (the input magnitudes are already in each stroke's length).
- Z is not summed: the swing height is the larger of the two paths' Z at that point, so a combined step lifts as high as either motion alone, never twice as high.
- With one input at zero its displacement is zero, so the result equals the other path exactly: translate-only and rotate-only behaviour, and their existing tests, are unchanged.
- `k` is the stride limit: `k = min(1, L / |stroke|)`, where `stroke` is the combined ground stroke (lift end to propel end) in the leg's XY plane and `L` the gait's `maxStrideLength`. Both motions shrink together, so the arc's radius is kept and only the speed drops.
- The phase capture points (`liftEnd`, `propelEnd`) use the same formula, so a phase change starts from where the previous segment ended, as now.

*Alternative:* keep the blend and remove the normalisation (`/ weightSum`). Rejected: the paths are absolute positions, so summing them without subtracting the stance point doubles the stance offset.

### D2. Reachability stays the guard

The combined target goes through `kinematics::isValidTarget` like every other target; a rejected target holds the leg and reports once, as today. `k` makes rejections rare by keeping the stroke within the gait's limit, and a host test sweeps forward × turn × gait × height for reachable targets.

### D3. Steer mode on the bridge

- Steer mode (default): `translate = (0, leftY)`, `rotate = leftX × sign(leftY ≥ 0 ? 1 : −1) + rightX`, clamped to ±100. The sign flip gives the car convention when reversing. With the left stick centred in Y, left X turns on the spot.
- Crab mode: today's mapping.
- `b` toggles; the mode is kept across disconnects with height, speed, gait and stride. The robot sees only translate and rotate, so it needs no change for this part.
- The stick dead zone applies per stick as now.

### D4. Status screen

The settings line shows `steer` or `crab` alongside gait and stride, within the existing 28-character lines.

## Risks / Trade-offs

- **Tighter turns at speed load the outer legs more.** The stride limit bounds the stroke, and power protection already covers the current.
- **The rotation stroke is an approximation** (leg-frame Y instead of the true tangent about the body centre). Arcs will be slightly off-radius; measured mounts fix this later without changing the spec.
- **Muscle memory:** the default stick layout changes. The crab layout is one press away.

## Migration Plan

Flash both boards (no protocol change, so the order does not matter). The README and the hardware-check skill describe the new default.
