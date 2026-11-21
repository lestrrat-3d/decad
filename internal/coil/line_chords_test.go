package coil_test

import (
	"math"
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
		{"wide spring annulus", spring, sectionrecord.Point2{U: 2}, sectionrecord.Point2{U: 4}},
		{"spring band", spring, sectionrecord.Point2{U: 3}, sectionrecord.Point2{U: 3, V: 1}},
		{"thread flank", thread, sectionrecord.Point2{U: 4.1}, sectionrecord.Point2{U: 5.3, V: 0.69}},
	} {
		t.Run(c.name, func(t *testing.T) {
			require.Nil(t, c.plan.Cuts(c.a, c.b))
		})
	}
	a, b := line(3, 0, 11, 0)
	require.Nil(t, coil.LinePlan{DV: 1, Side: 1, Pitch: 100, Step: 0.001}.Cuts(a, b),
		"a profile on the plan's negative side states no radii and keeps one chord")
}

// TestLinePlanCutsLongRadialLines checks that a radial line whose single
// cell's §5.4 departure passes max(LineChordTarget, 2·sag) is cut into chords
// whose every cell's exact departure (coil.CellDepartureUpper over the exact
// chord radii) meets that target,
// whose every radial run stays within its inner radius, and which grow
// outward as the twist term's pitch/ρ_min shrinks.
func TestLinePlanCutsLongRadialLines(t *testing.T) {
	for _, c := range []struct {
		name               string
		r0, r1, rise, span float64
		capped             float64
	}{
		{"shallow ramp", 3, 11, 4, 32.5 / 360, 1},
		{"steep face", 3, 11, 4, 3.5 / 360, 2},
		{"wide shallow ramp", 2.5, 14, 4, 32.5 / 360, 1},
		{"wide steep face", 2.5, 14, 4, 3.5 / 360, 2},
		{"steep tool past the face", 2.7, 11.3, 4 * 5.5 / 3.5, 5.5 / 360, 2},
		{"near the axis", 0.5, 3, 3, 2, 1},
	} {
		t.Run(c.name, func(t *testing.T) {
			plan, step := vAxisPlan(c.rise, c.span)
			pitch := new(big.Rat).SetFloat64(plan.Pitch)
			cuts := plan.Cuts(sectionrecord.Point2{U: c.r0}, sectionrecord.Point2{U: c.r1})
			require.NotEmpty(t, cuts)
			require.LessOrEqual(t, len(cuts)+1, coil.MaxLineChords)
			r0, r1 := new(big.Rat).SetFloat64(c.r0), new(big.Rat).SetFloat64(c.r1)
			params := []*big.Rat{new(big.Rat)}
			for _, f := range cuts {
				params = append(params, new(big.Rat).SetFloat64(f))
			}
			params = append(params, big.NewRat(1, 1))
			at := func(f *big.Rat) *big.Rat {
				return new(big.Rat).Add(r0, new(big.Rat).Mul(f, new(big.Rat).Sub(r1, r0)))
			}
			h := math.Pi * plan.Step
			targetF := math.Max(coil.LineChordTarget, c.r1*h*h)
			if len(cuts)+1 > coil.MaxLineChords-8 {
				// The ceiling raised the target to the smallest that fits it.
				targetF *= c.capped
			}
			target := new(big.Rat).SetFloat64(targetF * (1 + 1e-9))
			for k := range len(params) - 1 {
				a, b := at(params[k]), at(params[k+1])
				require.Positive(t, b.Cmp(a), "chord %d runs outward", k)
				dep, ok := coil.CellDepartureUpper(coil.Point(a), coil.Point(b), pitch, step)
				require.True(t, ok)
				require.LessOrEqual(t, dep.Cmp(target), 0, "chord %d of %d meets the target %v", k, len(params)-1, targetF)
				require.LessOrEqual(t, new(big.Rat).Sub(b, a).Cmp(a), 0, "chord %d's run stays within its inner radius", k)
			}
			first := new(big.Rat).Sub(params[1], params[0])
			inner := new(big.Rat).Sub(params[len(params)-2], params[len(params)-3])
			require.Positive(t, inner.Cmp(first), "the outer chords are longer than the innermost")
			whole, ok := coil.CellDepartureUpper(coil.Point(r0), coil.Point(r1), pitch, step)
			require.True(t, ok)
			require.Positive(t, whole.Cmp(new(big.Rat).SetFloat64(targetF)), "one chord misses the target")
		})
	}
}

// TestLoopsChordsLongLines checks the station chain of a chorded rectangle:
// every recorded corner is held exactly, every generated station sits within
// its plane round of the exact point at its cut's walk parameter, no line chord
// carries a sagitta, and Corners lists the recorded corners alone.
func TestLoopsChordsLongLines(t *testing.T) {
	loop := rectLoop(3, 11, -5.4, 0)
	plan, _ := vAxisPlan(4, 3.5/360)
	prof, err := coil.Loops(loop, nil, 16, plan)
	require.NoError(t, err)
	require.Len(t, prof.Segments, 4)
	radial := len(plan.Cuts(sectionrecord.Point2{U: 3}, sectionrecord.Point2{U: 11})) + 1
	require.Greater(t, radial, 1)
	inward := len(plan.Cuts(sectionrecord.Point2{U: 11}, sectionrecord.Point2{U: 3})) + 1
	require.Equal(t, []int{radial, 1, inward, 1},
		[]int{prof.Segments[0].Chords, prof.Segments[1].Chords, prof.Segments[2].Chords, prof.Segments[3].Chords})
	require.Len(t, prof.Pts, radial+inward+2)
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
		cuts := plan.Cuts(start, end)
		require.Len(t, cuts, seg.Chords-1)
		for k := 1; k < seg.Chords; k++ {
			v := seg.First + k
			require.Equal(t, si, prof.Chord[v].Segment)
			f := new(big.Rat).SetFloat64(cuts[k-1])
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

// TestLinePlanHonoursItsChordCeiling checks that a plan whose MaxChords sits
// below the chords its target needs cuts the line into at most MaxChords
// chords, and that a ceiling below two keeps one chord.
func TestLinePlanHonoursItsChordCeiling(t *testing.T) {
	plan, _ := vAxisPlan(4, 3.5/360)
	a, b := sectionrecord.Point2{U: 3}, sectionrecord.Point2{U: 11}
	require.Greater(t, len(plan.Cuts(a, b))+1, 16)
	plan.MaxChords = 16
	cuts := plan.Cuts(a, b)
	require.NotEmpty(t, cuts)
	require.LessOrEqual(t, len(cuts)+1, 16)
	plan.MaxChords = 1
	require.Nil(t, plan.Cuts(a, b))
}
