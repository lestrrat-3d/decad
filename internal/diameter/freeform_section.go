package diameter

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// FreeformSection reads a lower diameter bound from analytic walk ends and
// converted free-form span ends at both cap heights. lift places a
// plane-local (u, v), held within bound of the point the record denotes, at
// level z, and returns the held world point beside its proven distance from
// that denoted point: bound carried through the frame and placement, plus
// the lift's own rounding. The widest such distance is charged before
// publication.
func FreeformSection(ctx context.Context, outer sectionrecord.LoopRecord, holes []sectionrecord.LoopRecord,
	z0, z1, sectionDelta float64, lift func(u, v, z float64, bound proofbound.WalkEndBound) (r3.Vec, float64),
	axialDelta func() float64,
) (float64, bool, error) {
	work := freeform.NewFreeformWork()
	sawFreeform := false
	ownBound := 0.0
	var pts []r3.Vec

	addWitness := func(u, v float64, bound proofbound.WalkEndBound) bool {
		if proofbound.IsNonFinite(proofbound.WalkEndBoundAllow(bound)) {
			return false
		}
		for _, z := range [2]float64{z0, z1} {
			held, gap := lift(u, v, z, bound)
			if !usableMagnitude(gap) {
				return false
			}
			ownBound = math.Max(ownBound, gap)
			pts = append(pts, held)
		}
		return true
	}

	for _, loop := range append([]sectionrecord.LoopRecord{outer}, holes...) {
		for _, seg := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
			seg, err := sectionrecord.NormalizeSegment(seg)
			if err != nil {
				// Normalization reads no ctx; its refusal means this arm has
				// no witness for the recorded segment.
				return 0, false, nil //nolint:nilerr // structural refusal, not cancellation
			}
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				// WalkOf reads the record's work counter, not ctx; its
				// refusal likewise withholds this arm's witness set.
				return 0, false, nil //nolint:nilerr // structural refusal, not cancellation
			}
			if w.Kind != survey2d.WalkFreeform {
				if !addWitness(w.StartU, w.StartV, w.StartBound) || !addWitness(w.EndU, w.EndV, w.EndBound) {
					return 0, false, nil
				}
				continue
			}
			sawFreeform = true
			for _, span := range w.Spans {
				if len(span) == 0 {
					return 0, false, nil
				}
				for _, cp := range [2]freeform.RatPoint{span[0], span[len(span)-1]} {
					held, ok := splinebezier.Point2Of(cp)
					if !ok {
						return 0, false, nil
					}
					bound := proofbound.WalkEndBound{
						U: proofarith.RationalFloatError(cp.U, held.U),
						V: proofarith.RationalFloatError(cp.V, held.V),
					}
					if !addWitness(held.U, held.V, bound) {
						return 0, false, nil
					}
				}
			}
		}
	}
	if !sawFreeform {
		return 0, false, nil
	}

	d, ok, err := PointsWithBudget(proofbound.NewWorkBudget(ctx), pts)
	if err != nil {
		return 0, false, err
	}
	if !ok {
		return 0, false, nil
	}
	displacement := proofbound.AbsSumUpper(sectionDelta, axialDelta(), ownBound)
	d, ok = LowerForDisplacement(d, displacement)
	return d, ok, nil
}
