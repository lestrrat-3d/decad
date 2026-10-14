package capcontour

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// displacementBeforeCarrierReuse freezes the miter proof's prior
// evaluation order, including a separate CarrierOver call at each join.
func displacementBeforeCarrierReuse(walks []survey2d.SideWalk, joins []Join, amounts []float64, dDelta float64, keepZero bool) (float64, bool) {
	spans := make([]proofbound.RatInterval, len(walks))
	for i, a := range amounts {
		width := dDelta
		if keepZero && a == 0 {
			width = 0
		}
		span, ok := OffsetSpan(a, width)
		if !ok {
			return 0, false
		}
		spans[i] = span
	}
	delta := 0.0
	for i, w := range walks {
		if !w.IsCircular() {
			continue
		}
		inside := 1.0
		if w.Th1 < w.Th0 {
			inside = -1
		}
		held := w.Radius - inside*amounts[i]
		exact, ok := OffsetCircleRadius(w, spans[i])
		if !ok {
			return 0, false
		}
		delta = math.Max(delta, proofbound.IntervalFloatError(exact, held))
	}
	n := len(walks)
	for i, j := range joins {
		if n == 0 {
			break
		}
		p := (i + n - 1) % n
		prev, cur := walks[p], walks[i]
		if j.Arc || j.G1 {
			a, okA := joinFoot(prev, true, spans[p])
			b, okB := joinFoot(cur, false, spans[i])
			if !okA || !okB {
				return 0, false
			}
			if j.Arc {
				delta = math.Max(delta, math.Max(a.Reach(j.PA.U, j.PA.V), b.Reach(j.PB.U, j.PB.V)))
				continue
			}
			delta = math.Max(delta, Union(a, b).Reach(j.M.U, j.M.V))
			continue
		}
		ca, okA := CarrierOver(prev, spans[p])
		cb, okB := CarrierOver(cur, spans[i])
		if !okA || !okB {
			return 0, false
		}
		cands, ok := Intersect(ca, cb)
		if !ok {
			return 0, false
		}
		enc, ok := Nearest(cands, j.VU, j.VV)
		if !ok {
			return 0, false
		}
		delta = math.Max(delta, enc.Reach(j.M.U, j.M.V))
	}
	if proofbound.IsNonFinite(delta) {
		return 0, false
	}
	return delta, true
}

func lineLoop(points []sectionrecord.Point2) ([]survey2d.SideWalk, []Join) {
	walks := make([]survey2d.SideWalk, len(points))
	joins := make([]Join, len(points))
	for i, p := range points {
		q := points[(i+1)%len(points)]
		walks[i].Kind = survey2d.WalkLine
		walks[i].StartU, walks[i].StartV = p.U, p.V
		walks[i].EndU, walks[i].EndV = q.U, q.V
		joins[i] = Join{VU: p.U, VV: p.V, M: p, PA: p, PB: p}
	}
	return walks, joins
}

func TestDisplacementCarrierReuseMatchesPriorProof(t *testing.T) {
	t.Parallel()
	const miter = "miter"
	near := math.Nextafter(1, 2)
	large := math.Ldexp(1, 50)
	cases := []struct {
		name    string
		points  []sectionrecord.Point2
		amounts []float64
		width   float64
		zero    bool
		join    string
	}{
		{"square", []sectionrecord.Point2{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, []float64{1, 1, 1, 1}, 0, false, miter},
		{"reverse", []sectionrecord.Point2{{0, 10}, {10, 10}, {10, 0}, {0, 0}}, []float64{-1, -1, -1, -1}, 0, false, miter},
		{"oblique", []sectionrecord.Point2{{0, 0}, {6, 1}, {9, 8}, {2, 11}}, []float64{0.5, 1, 1.5, 2}, 0.125, true, miter},
		{"mixed joins", []sectionrecord.Point2{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, []float64{0, 1, 0, 1}, 0.125, true, "mixed"},
		{"all feet", []sectionrecord.Point2{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, []float64{1, 1, 1, 1}, 0, false, "feet"},
		{"rounding", []sectionrecord.Point2{{near, 0}, {10, near}, {near, 10}, {0, near}}, []float64{near, near, near, near}, math.SmallestNonzeroFloat64, false, miter},
		{"large dyadic", []sectionrecord.Point2{{large, 0}, {large + 1, near}, {large + 2, 3}, {large - 1, 4}}, []float64{near, 0.5, 0, 1.25}, math.SmallestNonzeroFloat64, true, miter},
		{"near parallel cancellation", []sectionrecord.Point2{{0, 0}, {10, 0}, {20, math.Ldexp(1, -45)}, {20, 10}}, []float64{0.25, -0.25, 0.25, -0.25}, 0, false, miter},
		{"parallel refusal", []sectionrecord.Point2{{0, 0}, {10, 0}, {20, 0}, {20, 10}}, []float64{1, 1, 1, 1}, 0, false, miter},
		{"zero length refusal", []sectionrecord.Point2{{0, 0}, {0, 0}, {10, 0}, {0, 10}}, []float64{1, 1, 1, 1}, 0, false, miter},
		{"invalid span", []sectionrecord.Point2{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, []float64{1, 1, 1, 1}, -1, false, miter},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			walks, joins := lineLoop(tc.points)
			for i := range joins {
				if tc.join == "feet" || tc.join == "mixed" && i == 0 {
					joins[i].Arc = true
				}
				if tc.join == "mixed" && i == 2 {
					joins[i].G1 = true
				}
			}
			beforeWalks := append([]survey2d.SideWalk(nil), walks...)
			beforeJoins := append([]Join(nil), joins...)
			beforeAmounts := append([]float64(nil), tc.amounts...)
			want, wantOK := displacementBeforeCarrierReuse(walks, joins, tc.amounts, tc.width, tc.zero)
			got, gotOK := displacementOver(walks, joins, tc.amounts, tc.width, tc.zero)
			require.Equal(t, wantOK, gotOK, "prior=%g, reused=%g", want, got)
			require.Equal(t, math.Float64bits(want), math.Float64bits(got), "prior=%g, reused=%g", want, got)
			switch tc.name {
			case "square", "reverse", "oblique", "mixed joins", "all feet":
				require.True(t, gotOK, "the fixture must reach a certified join proof")
			case "parallel refusal", "zero length refusal", "invalid span":
				require.False(t, gotOK, "the degenerate fixture must be refused")
			}
			require.Equal(t, beforeWalks, walks, "the proof changed its walks")
			require.Equal(t, beforeJoins, joins, "the proof changed its joins")
			require.Equal(t, beforeAmounts, tc.amounts, "the proof changed its amounts")
		})
	}
}

func TestDisplacementCarrierReuseMixedWalls(t *testing.T) {
	t.Parallel()
	points := []sectionrecord.Point2{{0, 0}, {10, 0}, {10, 10}, {0, 10}}
	for _, reverse := range []bool{false, true} {
		walks, joins := lineLoop(points)
		arc := sectionrecord.ArcSeg{
			Center: sectionrecord.Point2{U: 5}, Start: points[0], End: points[1], TEnd: 1,
		}
		if reverse {
			arc.TStart, arc.TEnd = 1, 0
		}
		w, err := boundarywalk.WalkOf(arc, freeform.NewFreeformWork())
		require.NoError(t, err)
		walks[0].SegmentWalk = w
		amounts := []float64{0.25, 0.25, 0.25, 0.25}
		for _, invalidRadius := range []bool{false, true} {
			if invalidRadius {
				walks[0].RadiusBound = math.Inf(1)
			}
			want, wantOK := displacementBeforeCarrierReuse(walks, joins, amounts, 0, false)
			got, gotOK := displacementOver(walks, joins, amounts, 0, false)
			require.Equal(t, wantOK, gotOK, "reverse=%v invalidRadius=%v", reverse, invalidRadius)
			require.Equal(t, math.Float64bits(want), math.Float64bits(got),
				"reverse=%v invalidRadius=%v", reverse, invalidRadius)
			if invalidRadius {
				require.False(t, gotOK, "a non-finite circular radius bound must be refused")
			} else {
				require.True(t, gotOK, "the mixed line-circle miter must be certified")
			}
		}
	}
}

func BenchmarkDisplacementMiterCarriers(b *testing.B) {
	walks, joins := lineLoop([]sectionrecord.Point2{{0, 0}, {10, 0}, {10, 10}, {0, 10}})
	b.ReportAllocs()
	for range b.N {
		if _, ok := Displacement(walks, joins, 1, 0); !ok {
			require.FailNow(b, "the square's carrier proof refused")
		}
	}
}
