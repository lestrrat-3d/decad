package diameter

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/r3"
)

// PrismStationWitnesses lifts the stations SectionStations lists on pp's
// walls to both z0 and z1. allow is
// the widest proven distance from a held point to the point of pp's recorded
// walls it stands for: the station's own gap carried through the frame and
// placement by Payload.PointBoundWith, together with the lift's own rounding.
//
// read is false, with no error, wherever SectionStations' is, and for a lift
// whose gap the proof cannot state. Cancellation through budget returns the
// error.
func PrismStationWitnesses(budget *proofbound.WorkBudget, pp prismrecord.Payload, work *freeform.FreeformWork) ([]r3.Vec, float64, bool, error) {
	return PrismStationWitnessesMemo(budget, pp, work, nil)
}

// PrismStationWitnessesMemo is PrismStationWitnesses reading the section's
// stations through memo, which a call shares across the prisms that place one
// profile. A nil memo reads them fresh. The stations are plane-local, so the
// placement never enters the memo's key; the lift is read fresh every time.
func PrismStationWitnessesMemo(budget *proofbound.WorkBudget, pp prismrecord.Payload, work *freeform.FreeformWork,
	memo *StationMemo) ([]r3.Vec, float64, bool, error) {
	stations, ok, err := memo.SectionStations(budget, append([]sectionrecord.LoopRecord{pp.Profile.Outer}, pp.Profile.Holes...), work)
	if err != nil || !ok {
		return nil, 0, false, err
	}
	pts := make([]r3.Vec, 0, 2*len(stations))
	allow := 0.0
	factor := pp.LiftFactor()
	for _, s := range stations {
		for _, z := range [2]float64{pp.Z0, pp.Z1} {
			a := pp.PointBoundWith(factor, proofbound.MeasuredScalar(s.U, s.Bound.U), proofbound.MeasuredScalar(s.V, s.Bound.V),
				proofbound.MeasuredScalar(z, 0))

			if !tolerance.UsableMagnitude(a) {
				return nil, 0, false, nil
			}
			allow = math.Max(allow, a)
			pts = append(pts, pp.Point(s.U, s.V, z))
		}
	}
	return pts, allow, true, nil
}

// PrismGateDiameter reads a diameter from the station witnesses of prisms.
// ok is false, with no error, when any prism's stations cannot be read.
func PrismGateDiameter(budget *proofbound.WorkBudget, prisms []prismrecord.Payload, displacement float64) (float64, bool, error) {
	g, ok, err := PrismGatePoints(budget, prisms, displacement)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.Diameter(budget)
}

// PrismGatePoints lists the station witnesses of prisms
// (PrismStationWitnesses). Each station is held within a proven gap of a
// point of its prism's recorded walls, and the recorded walls lie within
// displacement of the walls the payload denotes, so allow is displacement
// plus the widest gap. ok is false, with no error, when any prism's stations
// cannot be read.
//
// The carrier witnesses clearance.BuildPrismBodyCarriers places (CFace.Wit) are float samples
// with no proven gap to the body: a placed prism's held carrier maximum reads
// above the body's own diameter. This reading never reads them. Each one sits between
// two stations at its own angle, at a station, or inside the section.
//
// Every prism handed in must have walls that run the full height from z0 to
// z1 on the body (or z0 equal to z1, with its section's boundary on the
// body at that level), so that each station is a point of the body (within
// its gap and displacement).
func PrismGatePoints(budget *proofbound.WorkBudget, prisms []prismrecord.Payload, displacement float64) (GatePoints, bool, error) {
	return PrismGatePointsMemo(budget, prisms, displacement, nil)
}

// PrismGatePointsMemo is PrismGatePoints reading section stations through
// stations; nil reads them fresh.
func PrismGatePointsMemo(budget *proofbound.WorkBudget, prisms []prismrecord.Payload, displacement float64,
	stations *StationMemo) (GatePoints, bool, error) {
	work := freeform.NewFreeformWork()
	var g GatePoints
	for _, pp := range prisms {
		points, stationAllow, read, err := PrismStationWitnessesMemo(budget, pp, work, stations)
		if err != nil || !read {
			return GatePoints{}, false, err
		}
		g.Join(GatePoints{Points: points, Allow: stationAllow})
	}
	g.Allow = proofbound.AbsSumUpper(displacement, g.Allow)
	return g, true, nil
}

// PrismFreeformSection is pairkernel.BodyGateDiameter's arm for a free-form-walled
// prismPayload (agent-docs/verification-design.md §3): the clearance kernel's exact
// carrier model has no arm for a NURBSSurface side face any more than it has
// one for a displaced section, and gatewitness.Prism gives this payload none
// either, because its own sectionDelta reads zero — the one value that arm
// treats as "the section is already its own denotation, read newBodyGeomBudget
// instead". A free-form wall is not read through either model, so this
// function builds its own reference.
//
// It reports a certified LOWER bound on the body's own diameter: the maximum
// distance over a finite set of points KNOWN TO LIE ON the body — every
// analytic walk's own two endpoints, and every free-form span's own two
// endpoints (agent-docs/spline-design.md §5.1's exact-rational Bézier conversion,
// never the recorded control net, and for a FitSplineSeg never the raw
// recorded Fit points, which are neither the converted chain's own ends nor a
// hull the curve stays inside) — at both cap heights. A Bézier interpolates
// its end control points exactly, so every span endpoint is a real point of
// the curve itself, and every distance the maximum ranges over is therefore
// realized between two real body points.
//
// That is the witness set's own half of the claim, and it is only half: a
// maximum over real body points is at or below the true diameter as a
// QUANTITY, while what this arm publishes is a float64. The other half belongs
// to the shared reader — PointsWithBudget computes the winning pair's
// distance over exact rationals and rounds it toward zero — so the published
// number is at or below that maximum too. Composed, the reading can only
// UNDERSTATE the true diameter and never overstate it, exactly the direction
// §3 requires; the displacement subtracted below only widens the
// understatement further.
//
// The displacement subtracted from that maximum composes three terms, none of
// them a certificate claim: the section's own sectionDelta (zero for a
// free-form wall in practice, since the analytic prism-boolean reduction
// never admits one — agent-docs/prism-boolean-design.md's G4 — but read here rather
// than assumed), the payload's own axialDelta, and the widest per-witness
// gap: the endpoint bound of whichever walk produced it — an analytic walk's
// recorded-coordinate bound (zero for a whole segment, nonzero for a trimmed
// one) or a free-form span's own conversion rounding — carried through the
// frame and placement by Payload.PointBoundWith, together with the lift's own
// rounding.
// Composing the widest witness bound as one uniform displacement, rather than
// a bound per point, is the same convention LowerForDisplacement's
// other callers already use: every witness pair is presumed to move by up to
// that much, so the subtracted amount is twice the WORST one, never a mix.
//
// This arm PUBLISHES only when its own witness conversion and the shared reader
// both succeed; otherwise it withholds the diameter outright and never
// substitutes a weaker one. This comment owns the complete list of the paths it
// withholds on — agent-docs/verification-design.md §3 states the contract and points
// here rather than keeping a second copy:
//
//   - the profile carries no free-form segment at all, so this arm has nothing
//     to read the exact carrier model or gatewitness.Prism's own
//     displaced-section arm does not already read;
//   - a recorded segment normalizeSegment refuses, or walkOf refuses (an
//     R-table sentinel), so the section never becomes a walk at all;
//   - a free-form walk holds an empty Bézier span, or a span endpoint with no
//     finite float form (point2Of), so no witness can be placed on that curve;
//   - a witness's own endpoint bound cannot be derived (proofbound.WalkEndBoundAllow's
//     +Inf), since an absent bound must never read as a small one — this covers
//     an analytic walk's own two endpoints and a free-form span's alike;
//   - the shared reader declines the witness maximum (PointsWithBudget
//     answering ok=false: an empty set, a pair distance that is not a usable
//     magnitude, or a winning pair with no exact rational form);
//   - the displacement subtraction collapses the reading to non-positive
//     (LowerForDisplacement).
//
// A withheld answer is not rescued downstream. pairkernel.BodyGateDiameter falls through to
// fallbackGateDiameter, whose gatewitness.Prism has no arm for a prismPayload
// whose sectionDelta is zero, so the body ends with NO gate diameter and its
// bounded readings read Suspect. That is the sound direction to fail in — an
// absent reference admits nothing — but it is a real outcome of this arm, not
// one the arm's existence rules out.
//
// Every phase of this arm is cancellable, because neither of its two phases is
// bounded by a work counter of its own: the segment loop polls ctx before each
// segment, and the witness maximum polls it through PointsContext,
// the same reader the loftPayload arm above uses. That second poll is the one
// that matters for cost — the witness count grows with the profile's segment
// count (four points per segment), and the maximum is quadratic in it, so an unpolled scan is by
// far the longest thing a cancelled Verify could be left waiting on here. The
// resulting error is returned AS an error: cancellation is never folded into
// this arm's structural (0, false, nil) answer, which states only that the
// recorded section gives this arm nothing to read.
func PrismFreeformSection(ctx context.Context, pp prismrecord.Payload, axialDelta func() float64) (float64, bool, error) {
	factor := pp.LiftFactor()
	lift := func(u, v, z float64, bound proofbound.WalkEndBound) (r3.Vec, float64) {
		gap := pp.PointBoundWith(factor, proofbound.MeasuredScalar(u, bound.U), proofbound.MeasuredScalar(v, bound.V),
			proofbound.MeasuredScalar(z, 0))

		return pp.Point(u, v, z), gap
	}
	return FreeformSection(ctx, pp.Profile.Outer, pp.Profile.Holes,
		pp.Z0, pp.Z1, pp.SectionDelta, lift, axialDelta)
}
