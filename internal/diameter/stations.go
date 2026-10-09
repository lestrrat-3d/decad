package diameter

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// StationStep is the largest angle between consecutive diameter stations on a
// circular wall: fifteen degrees.
const StationStep = math.Pi / 12

// Station is a plane-local point on one recorded wall, held within Bound of
// the point the record denotes at the same parameter.
type Station struct {
	U, V  float64
	Bound proofbound.WalkEndBound
}

// SectionStations reads points on every wall for a lower diameter bound.
// Lines contribute both ends. Circular walls contribute recorded-parameter
// stations whose held sine and cosine carry a proven coordinate gap. Stations
// cover the sweep at StationStep spacing and include antipodes past half a turn.
// The usual carrier witnesses can miss a wall's diameter: three points on a
// 240-degree arc are only 2R·sin(120 degrees) apart, while the wall spans 2R.
// Each station's parameter is an exact rational fraction of the recorded
// range. CircularPointBound encloses the gap from the held sine and cosine
// to the point the record denotes at that parameter.
// A segment that cannot be read returns false; cancellation returns its error.
func SectionStations(budget *proofbound.WorkBudget, loops []sectionrecord.LoopRecord,
	work *freeform.FreeformWork) ([]Station, bool, error) {
	var out []Station
	add := func(u, v float64, bound proofbound.WalkEndBound) bool {
		if !proofbound.FiniteVec(r3.NewVec(u, v, 0)) || proofbound.IsNonFinite(bound.U) || proofbound.IsNonFinite(bound.V) {
			return false
		}
		out = append(out, Station{U: u, V: v, Bound: bound})
		return true
	}
	for _, loop := range loops {
		for _, recorded := range loop.Segments {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			seg, err := sectionrecord.NormalizeSegment(recorded)
			if err != nil {
				return nil, false, nil //nolint:nilerr // structural refusal withholds the station reading
			}
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, false, nil //nolint:nilerr // structural refusal withholds the station reading
			}
			switch w.Kind {
			case survey2d.WalkLine:
				if !add(w.StartU, w.StartV, w.StartBound) || !add(w.EndU, w.EndV, w.EndBound) {
					return nil, false, nil
				}
				continue
			case survey2d.WalkCircular:
			default:
				return nil, false, nil
			}
			start, span, ok := loftmesh.CircularSegmentRange(seg)
			if !ok {
				return nil, false, nil
			}
			for _, frac := range stationFractions(math.Abs(w.Th1 - w.Th0)) {
				if err := budget.Step(); err != nil {
					return nil, false, err
				}
				f, _ := frac.Float64()
				sin, cos := math.Sincos(w.Th0 + f*(w.Th1-w.Th0))
				u, v := w.CU+w.Radius*cos, w.CV+w.Radius*sin
				t := new(big.Rat).Add(start, new(big.Rat).Mul(frac, span))
				if !add(u, v, boundarywalk.CircularPointBound(seg, t, u, v)) {
					return nil, false, nil
				}
			}
		}
	}
	return out, true, nil
}

// ChainWalkEndpointAllow covers computed walk endpoint coordinates omitted
// from a chain body's topology-vertex bounds. It reads every source segment,
// including segments later coalesced into a single wall.
func ChainWalkEndpointAllow(ctx context.Context, chains []sectionrecord.ChainRecord) (float64, bool, error) {
	work := freeform.NewFreeformWork()
	allow := 0.0
	for _, chain := range chains {
		for _, segment := range chain.Segments {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
			walk, err := boundarywalk.WalkOf(segment, work)
			if err != nil {
				return 0, false, nil //nolint:nilerr // structural walk refusal withholds the reference
			}
			for _, bound := range [2]proofbound.WalkEndBound{walk.StartBound, walk.EndBound} {
				endAllow := proofbound.WalkEndBoundAllow(bound)
				if !usableMagnitude(endAllow) {
					return 0, false, nil
				}
				allow = math.Max(allow, endAllow)
			}
			// An arc's recorded natural end can sit off the radius its
			// denoted circle reads from Start even with zero walk-end bound.
			residual := loftmesh.ArcNaturalEndRadialUpper(segment)
			if !usableMagnitude(residual) {
				return 0, false, nil
			}
			allow = math.Max(allow, residual)
		}
	}
	return allow, true, ctx.Err()
}

// stationFractions lists exact fractions of one circular wall's recorded
// parameter range. It includes both antipodes when the sweep exceeds half a turn.
func stationFractions(sweep float64) []*big.Rat {
	n := max(2, int(math.Ceil(sweep/StationStep)))
	n += n % 2
	fracs := make([]*big.Rat, 0, n+3)
	for k := 0; k <= n; k++ {
		fracs = append(fracs, big.NewRat(int64(k), int64(n)))
	}
	if sweep > math.Pi {
		opposite := math.Pi / sweep
		fracs = append(fracs, new(big.Rat).SetFloat64(opposite), new(big.Rat).SetFloat64(1-opposite))
	}
	return fracs
}
