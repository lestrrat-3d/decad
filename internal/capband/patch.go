package capband

import "github.com/lestrrat-3d/decad/internal/sectionrecord"

// Point is one plane-local cap patch coordinate.
type Point = sectionrecord.Point2

// Patch is one patch's exact analytic description, plane-local (u, v)
// plus the two axial levels, kept alongside the topology for the moments and
// tessellation passes (docs/modify-reach-design.md §8.4).
type Patch struct {
	Circular bool
	// Plane patch (Circular == false): the two side-level (original) points
	// and the two cap-level (offset) points, in walk order.
	SideA, SideB, CapA, CapB Point
	// Cone patch (Circular == true): concentric center, the two radii
	// (side = original wall radius, cap = offset radius) and the angular
	// EXTENT — always normalized to Th0 < Th1, never the walk's own sense,
	// since patchRawFlux carries the material side in its own sign
	// corrections and the DX7 survey reads an increasing window.
	CU, CV                float64
	SideRadius, CapRadius float64
	Th0, Th1              float64
	// SweepCCW records whether the patch's OWN angular walk runs
	// counter-clockwise (increasing theta). Th0/Th1 above are normalized to
	// Th0 < Th1 because the DX7 survey reads them as an increasing window,
	// and that normalization discards the walk's sense — so patchRawFlux
	// reads it from here: a clockwise-walked patch integrated over the
	// normalized window comes out with its surface facing the wrong way, and
	// its flux is negated. A reflex corner's apex patch is always clockwise
	// (the inward offset's connector arc runs pA -> pB with theta
	// decreasing), and a plane patch never reads this field at all — its own
	// walk lives in the SideA -> SideB vertex order.
	SweepCCW bool
	// WholeTurn records that this patch's angular window really is the FULL
	// period — the single closed circle a cornerless loop offsets into, the
	// one shape below that is built without a corner join. It is set from
	// that structural fact and never from a comparison of Th0 and Th1: fl(2π)
	// is not 2π, so no float window ever proves a full period. It is read by
	// capblend_moments.go's flux bound alone, where ∮cos = ∮sin = 0 over a
	// full period makes the eccentric term's TRUE value exactly zero, so the
	// whole held value is its own error and no Sincos magnitude envelope is
	// owed for it.
	WholeTurn bool

	// CapTh0, CapTh1 are the CAP-LEVEL directrix's own angular window,
	// normalized (and swapped, where Th0/Th1 are) the SAME way — CapTh0
	// paired with Th0's own corner, CapTh1 with Th1's. A regular wall's cap
	// contour is the offset arc TRIMMED at the mitered corner feet
	// (capWallFoot), which sit at a DIFFERENT angle than the wall's own
	// endpoints wherever the corner is a genuine (non-tangent) miter
	// (docs/modify-reach-design.md §8.3): Th0/Th1 above stay the wall's own
	// full recorded sweep — the SIDE directrix, which the DX7 survey reads
	// because the patch genuinely attains it there — while CapTh0/CapTh1 is
	// the trimmed CAP directrix, and patchRawFlux integrates the
	// straight-ruled patch BETWEEN the two windows rather than assume a
	// single rotationally-symmetric cone sector spanning one shared window.
	// patchAreaOf does not: its own area stays the constant-slant
	// frustum-sector formula read against the trimmed CAP window alone, with
	// the two windows genuinely differing widening its BOUND by the proven
	// corner-skew allowance (SkewStart/SkewEnd below) rather than its own
	// integral.
	// A reflex corner's apex patch (side radius zero, so no side angle
	// matters) and the single cornerless closed circle (no corner trims it
	// at all, so both windows are the identical full period) set
	// CapTh0/CapTh1 equal to Th0/Th1, which is what lets every Circular
	// patch route through the one general formula.
	CapTh0, CapTh1 float64

	SideZ, CapZ float64

	// ContourAllow is this ONE patch's own proven allowance for how far its
	// area can differ from the ruled quad the construction denotes, given the
	// cap-level directrix's own displacement (internal/proofbound/bounds.go's
	// proofbound.BandPatchAreaAllow) — computed once, at build time, from this patch's
	// own held chord length and slant distance, and added into patchAreaOf's
	// returned bound. It is zero wherever the band's own contour displacement
	// is zero (an axis-aligned section's exact miters), which is what leaves
	// patchAreaOf's Plane/Cone bound carrying only its own arithmetic and the
	// side level's allowance (LevelDelta below) there.
	ContourAllow float64

	// LevelDelta is the SIDE level's conversion and float-sum rounding: SideZ
	// is the single float sum CapZ + matSign*ds, so this patch's whole side
	// directrix sits that far from the level it denotes — the same term
	// capSlantEdge charges into a slant edge's length and capBandVolume charges
	// for the identical level.
	// patchAreaOf reads it as the axial half of its own displacement
	// allowance (internal/proofbound/bounds.go's proofbound.BandLevelAreaAllow), beside ContourAllow's
	// cap-level half; without it both of that function's arms would read the
	// side level as an exact input and bound only the patch they BUILT.
	LevelDelta float64

	// CapThAllow is the proven bound on |held (CapTh1−CapTh0) − true window|,
	// derived at build time from the same proofbound.Atan2Interval bracket capWallArcBound
	// builds for this wall's own cap-level arc (capSweepAllow,
	// capblend_contour.go), or from proofbound.PiLower/proofbound.PiUpper directly for the one
	// whole-turn circle, whose cap-level sweep is a structural fact of that
	// construction (WholeTurn) rather than an offset corner's own computed
	// feet. patchAreaOf's Cone arm reads it to bracket the frustum-sector
	// formula's Δθ factor; nothing else does.
	CapThAllow float64

	// SkewStart and SkewEnd are capband.CornerSkewUpper's proven bounds on the
	// exact angle between the side directrix's end and the cap directrix's end
	// at the (Th0, CapTh0) and (Th1, CapTh1) corners, read from the held
	// plane-local ends. patchAreaOf's Cone arm turns them into the ruled
	// patch's distance from the frustum sector it publishes. An apex patch
	// holds zero for both: its side directrix is the corner point, the same
	// point at every angle, so its rulings pair the cap arc with that point at
	// the cap arc's own angle.
	SkewStart, SkewEnd float64
}
