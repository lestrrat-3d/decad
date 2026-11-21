package coil_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// vAxisPlan is the line plan of a coil about the plane's V axis, its profile
// at positive U, rising rise over span turns at 256 stations per turn.
func vAxisPlan(rise, span float64) (coil.LinePlan, *big.Rat) {
	n, _ := coil.StationCount(new(big.Rat).SetFloat64(span), 256)
	step := new(big.Rat).Quo(new(big.Rat).SetFloat64(span), big.NewRat(n, 1))
	stepF, _ := step.Float64()
	return coil.LinePlan{DV: 1, Side: -1, Pitch: rise / span, Step: stepF}, step
}

func rectLoop(r0, r1, z0, z1 float64) sectionrecord.LoopRecord {
	pts := []sectionrecord.Point2{{U: r0, V: z0}, {U: r1, V: z0}, {U: r1, V: z1}, {U: r0, V: z1}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func TestLinePlanKeepsShortLinesWhole(t *testing.T) {
	line := func(u0, v0, u1, v1 float64) (sectionrecord.Point2, sectionrecord.Point2) {
		return sectionrecord.Point2{U: u0, V: v0}, sectionrecord.Point2{U: u1, V: v1}
	}
	spring, _ := vAxisPlan(3, 2)
	thread, _ := vAxisPlan(4.5, 3)
	for _, c := range []struct {
		name string
		plan coil.LinePlan
		a, b sectionrecord.Point2
	}{
		{"zero plan", coil.LinePlan{}, sectionrecord.Point2{U: 3}, sectionrecord.Point2{U: 11}},
		{"spring annulus", spring, sectionrecord.Point2{U: 2}, sectionrecord.Point2{U: 3}},
		{"spring band", spring, sectionrecord.Point2{U: 3}, sectionrecord.Point2{U: 3, V: 1}},
		{"thread flank", thread, sectionrecord.Point2{U: 4.1}, sectionrecord.Point2{U: 5.3, V: 0.69}},
	} {
		t.Run(c.name, func(t *testing.T) {
			require.Equal(t, 1, c.plan.Chords(c.a, c.b))
		})
	}
	a, b := line(3, 0, 11, 0)
	require.Equal(t, 1, coil.LinePlan{DV: 1, Side: 1, Pitch: 100, Step: 0.001}.Chords(a, b),
		"a profile on the plan's negative side states no radii and keeps one chord")
}

// TestLinePlanChordsLongRadialLines checks that a radial line whose single
// cell's §5.4 departure passes the target is cut into the fewest equal chords
// whose every cell's exact departure (coil.CellDepartureUpper over the exact
// chord radii) meets LineChordTargetScale times the larger radius, and whose
// every chord's radial run stays within its inner radius.
func TestLinePlanChordsLongRadialLines(t *testing.T) {
	for _, c := range []struct {
		name               string
		r0, r1, rise, span float64
		least              int
	}{
		{"shallow ramp", 3, 11, 4, 32.5 / 360, 2},
		{"steep face", 3, 11, 4, 3.5 / 360, 2},
		{"wide shallow ramp", 2.5, 14, 4, 32.5 / 360, 2},
		{"wide steep face", 2.5, 14, 4, 3.5 / 360, 2},
		{"near the axis", 0.5, 3, 3, 2, 5},
	} {
		t.Run(c.name, func(t *testing.T) {
			plan, step := vAxisPlan(c.rise, c.span)
			pitch := new(big.Rat).SetFloat64(plan.Pitch)
			m := plan.Chords(sectionrecord.Point2{U: c.r0}, sectionrecord.Point2{U: c.r1})
			require.GreaterOrEqual(t, m, c.least)
			require.Less(t, m, coil.MaxLineChords)
			r0, r1 := new(big.Rat).SetFloat64(c.r0), new(big.Rat).SetFloat64(c.r1)
			target := new(big.Rat).SetFloat64(coil.LineChordTargetScale * c.r1)
			at := func(k, m int) *big.Rat {
				f := big.NewRat(int64(k), int64(m))
				return new(big.Rat).Add(r0, new(big.Rat).Mul(f, new(big.Rat).Sub(r1, r0)))
			}
			worst := func(m int) *big.Rat {
				out := new(big.Rat)
				for k := range m {
					dep, ok := coil.CellDepartureUpper(coil.Point(at(k, m)), coil.Point(at(k+1, m)), pitch, step)
					require.True(t, ok)
					if dep.Cmp(out) > 0 {
						out = dep
					}
				}
				return out
			}
			require.LessOrEqual(t, worst(m).Cmp(target), 0, "every chord's cell meets the target")
			first := new(big.Rat).Sub(at(1, m), at(0, m))
			require.LessOrEqual(t, first.Cmp(r0), 0, "the innermost chord's run stays within its inner radius")
			whole, ok := coil.CellDepartureUpper(coil.Point(r0), coil.Point(r1), pitch, step)
			require.True(t, ok)
			require.Positive(t, whole.Cmp(target), "one chord misses the target")
			if m > 2 {
				fewer := worst(m - 1)
				shortRun := new(big.Rat).Sub(at(1, m-1), at(0, m-1)).Cmp(r0) <= 0
				require.False(t, fewer.Cmp(target) <= 0 && shortRun, "one chord fewer would meet the target")
			}
		})
	}
}

// TestLoopsChordsLongLines checks the station chain of a chorded rectangle:
// every recorded corner is held exactly, every generated station sits within
// its plane round of the exact point at its walk parameter k/m, no line chord
// carries a sagitta, and Corners lists the recorded corners alone.
func TestLoopsChordsLongLines(t *testing.T) {
	loop := rectLoop(3, 11, -5.4, 0)
	plan, _ := vAxisPlan(4, 3.5/360)
	prof, err := coil.Loops(loop, nil, 16, plan)
	require.NoError(t, err)
	require.Len(t, prof.Segments, 4)
	radial := plan.Chords(sectionrecord.Point2{U: 3}, sectionrecord.Point2{U: 11})
	require.Greater(t, radial, 1)
	require.Equal(t, []int{radial, 1, radial, 1},
		[]int{prof.Segments[0].Chords, prof.Segments[1].Chords, prof.Segments[2].Chords, prof.Segments[3].Chords})
	require.Len(t, prof.Pts, 2*radial+2)
	require.Len(t, prof.LoopIdx, 1)
	require.Len(t, prof.LoopIdx[0], len(prof.Pts))

	corners := prof.Corners()
	require.Len(t, corners, 1)
	require.Len(t, corners[0], 4)
	for i, want := range []sectionrecord.Point2{{U: 3, V: -5.4}, {U: 11, V: -5.4}, {U: 11, V: 0}, {U: 3, V: 0}} {
		require.Equal(t, want, prof.Pts[corners[0][i]])
		require.Zero(t, prof.Round[corners[0][i]])
	}
	for v, c := range prof.Chord {
		require.Nil(t, c.Sag, "station %d", v)
		require.Nil(t, c.Arc, "station %d", v)
	}
	for si, seg := range prof.Segments {
		s := seg.Record.(sectionrecord.LineSeg)
		start, end := s.Start, s.End
		if seg.Reversed {
			start, end = end, start
		}
		for k := 1; k < seg.Chords; k++ {
			v := seg.First + k
			require.Equal(t, si, prof.Chord[v].Segment)
			f := big.NewRat(int64(k), int64(seg.Chords))
			exact := func(a, b float64) *big.Rat {
				ra, rb := new(big.Rat).SetFloat64(a), new(big.Rat).SetFloat64(b)
				return new(big.Rat).Add(ra, new(big.Rat).Mul(f, new(big.Rat).Sub(rb, ra)))
			}
			du := new(big.Rat).Sub(new(big.Rat).SetFloat64(prof.Pts[v].U), exact(start.U, end.U))
			dv := new(big.Rat).Sub(new(big.Rat).SetFloat64(prof.Pts[v].V), exact(start.V, end.V))
			d2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
			r := new(big.Rat).SetFloat64(prof.Round[v])
			require.LessOrEqual(t, d2.Cmp(new(big.Rat).Mul(r, r)), 0, "station %d sits within its round", v)
		}
	}

	whole, err := coil.Loops(loop, nil, 16, coil.LinePlan{})
	require.NoError(t, err)
	require.Len(t, whole.Pts, 4)
	require.Equal(t, whole.LoopIdx, whole.Corners())
}
