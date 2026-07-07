# Getting Fingerprint Scanner → Holder Transform in SolidWorks

## Goal
Extract exact relative transform (translation + rotation) between fingerprint scanner and main holder, for use as `T_part_target` in pick-place pipeline.

## Method 1 — Measure Tool (quick check)
`Tools → Evaluate → Measure`
- Select scanner datum face/origin, then holder datum face/origin
- Returns delta X/Y/Z + angle
- Good for sanity checks, not clean export

## Method 2 — Coordinate Systems (recommended for USD export)
1. `Insert → Reference Geometry → Coordinate System`
2. Create CS1 at holder's origin (use existing planes/axes/datum)
3. Create CS2 at scanner's origin (same convention)
4. `Tools → Evaluate → Measure` between CS1 and CS2
5. Returns full relative transform: translation (X,Y,Z) + rotation matrix

**Why this method:** consistent frame convention transfers cleanly to USD, avoids reconstructing transform from arbitrary face picks.

## Method 3 — Mate Values (if applicable)
- If mate is distance/angle type (not coincident), FeatureManager mate dialog shows the numeric offset already used
- Direct read, no extra steps

## Next Steps
- Export CS1/CS2 transform values
- Verify pivot/units survive STEP AP242 → USD conversion (known issue in this pipeline)
- Use as `T_part_target` in: `T_gripper_target = T_part_target · (T_gripper_part)⁻¹`
