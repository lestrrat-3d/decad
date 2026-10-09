package tessellation

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// SectionPoint is a plane-local vertex of a chorded section.
type SectionPoint = sectionrecord.Point2

// LoopClearanceFailure names the first pair of distinct loops whose measured
// chord distance does not exceed their sagitta bounds plus the rounding floor.
type LoopClearanceFailure struct {
	LoopA, LoopB               int
	Distance, Floor, ChordGate float64
}

// WalkClearanceFailure names the first non-adjacent chord pair in one loop
// whose measured distance does not exceed its clearance gate.
type WalkClearanceFailure struct {
	Loop, ChordA, ChordB       int
	Distance, Floor, ChordGate float64
}

// SectionLoopClearance checks distinct chorded loops against their summed
// sagitta tubes and the section's translation-honest rounding floor.
func SectionLoopClearance(ctx context.Context, pts []SectionPoint, loopIdx [][]int, loopSag []float64) (LoopClearanceFailure, bool, error) {
	floor, err := sectionClearanceFloor(ctx, pts)
	if err != nil {
		return LoopClearanceFailure{}, false, err
	}
	for i := range loopIdx {
		for j := i + 1; j < len(loopIdx); j++ {
			chordGate := loopSag[i] + loopSag[j]
			gate := chordGate + floor
			d, err := loopPolylineDistance(ctx, pts, loopIdx[i], loopIdx[j])
			if err != nil {
				return LoopClearanceFailure{}, false, err
			}
			if d <= gate {
				return LoopClearanceFailure{LoopA: i, LoopB: j, Distance: d, Floor: floor, ChordGate: chordGate}, true, nil
			}
		}
	}
	return LoopClearanceFailure{}, false, nil
}

// sectionClearanceFloor combines 1e-9 of the section span with four ulps of
// its largest coordinate magnitude. The span is translation invariant; the
// ulp term charges rounding at the actual coordinate scale.
func sectionClearanceFloor(ctx context.Context, pts []SectionPoint) (float64, error) {
	minU, maxU := math.Inf(1), math.Inf(-1)
	minV, maxV := math.Inf(1), math.Inf(-1)
	maxAbs := 0.0
	budget := proofbound.NewWorkBudget(ctx)
	for _, p := range pts {
		if err := budget.Step(); err != nil {
			return 0, err
		}
		minU, maxU = math.Min(minU, p.U), math.Max(maxU, p.U)
		minV, maxV = math.Min(minV, p.V), math.Max(maxV, p.V)
		maxAbs = math.Max(maxAbs, math.Max(math.Abs(p.U), math.Abs(p.V)))
	}
	span := math.Hypot(maxU-minU, maxV-minV)
	return 1e-9*span + 4*(math.Nextafter(maxAbs, math.Inf(1))-maxAbs), nil
}

// SectionWalkClearance checks every non-adjacent chord pair within each loop.
// The summed sagittae bound each pair's analytic-to-chord homotopy; adjacent
// chords share a station and are decided by the loop's own turn.
// sag[i][k] bounds the chord leaving station loopIdx[i][k].
func SectionWalkClearance(ctx context.Context, pts []SectionPoint, loopIdx [][]int, sag [][]float64) (WalkClearanceFailure, bool, error) {
	floor, err := sectionClearanceFloor(ctx, pts)
	if err != nil {
		return WalkClearanceFailure{}, false, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	for i, idx := range loopIdx {
		m := len(idx)
		if m < 4 {
			continue
		}
		for a := range m {
			for b := a + 2; b < m; b++ {
				if err := budget.Step(); err != nil {
					return WalkClearanceFailure{}, false, err
				}
				if a == 0 && b == m-1 {
					continue
				}
				chordGate := sag[i][a] + sag[i][b]
				gate := chordGate + floor
				d := sectionSegmentDistance(
					pts[idx[a]], pts[idx[(a+1)%m]],
					pts[idx[b]], pts[idx[(b+1)%m]],
				)
				if d > gate {
					continue
				}
				return WalkClearanceFailure{Loop: i, ChordA: a, ChordB: b, Distance: d, Floor: floor, ChordGate: chordGate}, true, nil
			}
		}
	}
	return WalkClearanceFailure{}, false, nil
}

// loopPolylineDistance is the minimum distance between two closed sample
// polylines, charged to a fresh budget for each loop pair.
func loopPolylineDistance(ctx context.Context, pts []SectionPoint, a, b []int) (float64, error) {
	best := math.Inf(1)
	budget := proofbound.NewWorkBudget(ctx)
	for i := range a {
		a0, a1 := pts[a[i]], pts[a[(i+1)%len(a)]]
		for j := range b {
			if err := budget.Step(); err != nil {
				return 0, err
			}
			b0, b1 := pts[b[j]], pts[b[(j+1)%len(b)]]
			best = math.Min(best, sectionSegmentDistance(a0, a1, b0, b1))
		}
	}
	return best, nil
}

func sectionSegmentDistance(a0, a1, b0, b1 SectionPoint) float64 {
	if sectionSegmentsCross(a0, a1, b0, b1) {
		return 0
	}
	d := math.Min(sectionPointSegmentDistance(a0, b0, b1), sectionPointSegmentDistance(a1, b0, b1))
	d = math.Min(d, sectionPointSegmentDistance(b0, a0, a1))
	return math.Min(d, sectionPointSegmentDistance(b1, a0, a1))
}

func sectionSegmentsCross(a0, a1, b0, b1 SectionPoint) bool {
	d1 := sectionCross(b0, b1, a0)
	d2 := sectionCross(b0, b1, a1)
	d3 := sectionCross(a0, a1, b0)
	d4 := sectionCross(a0, a1, b1)
	return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
		((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))
}

func sectionCross(a, b, c SectionPoint) float64 {
	return (b.U-a.U)*(c.V-a.V) - (b.V-a.V)*(c.U-a.U)
}

func sectionPointSegmentDistance(p, s0, s1 SectionPoint) float64 {
	du, dv := s1.U-s0.U, s1.V-s0.V
	l2 := du*du + dv*dv
	t := 0.0
	if l2 > 0 {
		t = math.Min(1, math.Max(0, ((p.U-s0.U)*du+(p.V-s0.V)*dv)/l2))
	}
	return math.Hypot(p.U-(s0.U+t*du), p.V-(s0.V+t*dv))
}
