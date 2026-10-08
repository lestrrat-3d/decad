package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds every rim vertex a feature builds from a recorded region to
// the points its two neighbouring segments DENOTE there: each segment's entity
// evaluated at its own recorded parameter (docs/sketch-seam-design.md §1/§2,
// docs/evaluator-design.md §3/§4). A trimmed line's end is the exact lerp at
// its recorded t, a circle's end is its centre plus r·(cos 2πt, sin 2πt), and
// at a cut junction the two neighbours denote two different points. A vertex
// publishes one position and one bound, so the bound must reach every denoted
// end that meets there. Each check below takes the distance over math/big at
// 256 bits and compares it with the published bound.
//
// Shown-to-fail, one leg of boundarywalk.JunctionVertex at a time:
//   - With no walk-end term for lines and circles (the builders' old
//     charge), every fixture but the integer ones goes red: the cut caps
//     publish Exact vertices 1.8e-15 (1370 mm) and 9.9e-12 mm (6e5 mm) from a
//     denoted end, the rounded circle seam one 3.6e-16 mm from its point, and
//     the trimmed square corner one 5.5e-11 mm from the trimmed side's start.
//     The revolve, which charged its own walk start only, publishes 3.1e-15
//     at a vertex 6.1e-14 mm from the line's denoted end.
//   - Without the cross term (|held − prev's end| + prev's bound), the cut
//     caps, the revolve, the rounded seam and the merged run go red.
//   - Without the next walk's own start term, the trimmed corner, the
//     rounded seam and the 6e5 mm cap go red.
//   - Taking the larger term at a circle's own seam instead of the smaller
//     publishes a bound on the integer circle's exact seam vertex, and its
//     Exact leg goes red.
//   - Placing a clockwise hole's seam vertex at its t = 1 start rather than
//     its exact t = 0 end publishes the integer hole's seam with a 3.6e-15 mm
//     bound, and its Exact leg goes red.
//   - Leaving a merged wall with the dropped junction's end bound
//     (coalesceWalksWithPoll's merge) publishes the merged run's corner Exact
//     1.2e-16 mm from the trimmed base's denoted end.

// denotedSinCos evaluates sin and cos of x by their Taylor series
// at junctionPrec bits. |x| ≤ 2π here, so the series converges well inside the
// precision.
func denotedSinCos(x *big.Float) (*big.Float, *big.Float) {
	sin, cos := jf(0), jf(0)
	term := jf(1)
	tiny := new(big.Float).SetPrec(junctionPrec).SetMantExp(jf(1), -junctionPrec-16)
	for n := range int64(400) {
		if n > 0 {
			term.Mul(term, x)
			term.Quo(term, jf(float64(n)))
		}
		switch n % 4 {
		case 0:
			cos.Add(cos, term)
		case 1:
			sin.Add(sin, term)
		case 2:
			cos.Sub(cos, term)
		case 3:
			sin.Sub(sin, term)
		}
		if n > 8 && new(big.Float).Abs(term).Cmp(tiny) < 0 {
			break
		}
	}
	return sin, cos
}

type denotedPoint struct{ u, v *big.Float }

// denotedLerp is a + t·(b − a), exact at junctionPrec for float operands.
func denotedLerp(a, b, t float64) *big.Float {
	d := new(big.Float).SetPrec(junctionPrec).Sub(jf(b), jf(a))
	d.Mul(d, jf(t))
	return d.Add(d, jf(a))
}

// denotedEnds lists the points every recorded segment of a profile denotes
// at its two recorded parameters. A record with a segment kind this file does not
// evaluate fails the test rather than skipping the segment.
func denotedEnds(t *testing.T, record *decad.ProfileRecord) []denotedPoint {
	t.Helper()
	var segs []decad.CurveSegment
	for _, loop := range append([]decad.LoopRecord{record.Outer}, record.Holes...) {
		segs = append(segs, loop.Segments...)
	}
	return denotedSegmentEnds(t, segs)
}

// denotedSegmentEnds is denotedEnds over a plain segment list.
func denotedSegmentEnds(t *testing.T, segs []decad.CurveSegment) []denotedPoint {
	t.Helper()
	twoPi := new(big.Float).SetPrec(junctionPrec).Mul(junctionPi(), jf(2))
	var out []denotedPoint
	{
		for _, seg := range segs {
			switch s := seg.(type) {
			case decad.LineSeg:
				for _, at := range []float64{s.TStart, s.TEnd} {
					out = append(out, denotedPoint{denotedLerp(s.Start.U, s.End.U, at), denotedLerp(s.Start.V, s.End.V, at)})
				}
			case decad.CircleSeg:
				r, err := s.Radius.In(units.Millimeter)
				require.NoError(t, err)
				for _, at := range []float64{s.TStart, s.TEnd} {
					// t = 0 and t = 1 denote the angle 0 and a full turn, whose
					// sine and cosine are exactly 0 and 1. The series would leave
					// π's own 256-bit truncation there instead.
					sin, cos := jf(0), jf(1)
					if at != 0 && at != 1 {
						sin, cos = denotedSinCos(new(big.Float).SetPrec(junctionPrec).Mul(twoPi, jf(at)))
					}
					u := new(big.Float).SetPrec(junctionPrec).Mul(cos, jf(r))
					v := new(big.Float).SetPrec(junctionPrec).Mul(sin, jf(r))
					out = append(out, denotedPoint{u.Add(u, jf(s.Center.U)), v.Add(v, jf(s.Center.V))})
				}
			default:
				require.Failf(t, `unexpected segment kind`, `%T`, seg)
			}
		}
	}
	return out
}

// vertexReach is the exact distance from a vertex position to a denoted
// point in space, as a float64 rounded from junctionPrec bits.
func vertexReach(pos [3]float64, p [3]*big.Float) float64 {
	sum := jf(0)
	for i, c := range p {
		d := new(big.Float).SetPrec(junctionPrec).Sub(jf(pos[i]), c)
		sum.Add(sum, d.Mul(d, d))
	}
	reach, _ := sum.Sqrt(sum).Float64()
	return reach
}

// inPlane places a denoted point at the vertex's own height: an extrude's or a
// patch's vertex sits at a recorded plane point lifted to an exact level.
func inPlane(p denotedPoint, z float64) [][3]*big.Float {
	return [][3]*big.Float{{p.u, p.v, jf(z)}}
}

// requireVerticesReachDenoted checks every body vertex against every placed
// denoted segment end within 1e-6 mm of it, and returns how many vertices
// published a nonzero bound and how many met two or more denoted ends.
func requireVerticesReachDenoted(t *testing.T, body *decad.Body, ends []denotedPoint, place func(denotedPoint, float64) [][3]*big.Float) (int, int) {
	t.Helper()
	bounded, shared := 0, 0
	for _, vtx := range body.Vertices() {
		pos := vtx.Position()
		bound, err := pos.Bound.In(units.Millimeter)
		require.NoError(t, err)
		at := [3]float64{pos.Value.X, pos.Value.Y, pos.Value.Z}
		if bound > 0 {
			bounded++
			require.Equal(t, decad.Approximate, pos.Exactness)
		} else {
			require.Equal(t, decad.Exact, pos.Exactness)
		}
		met := 0
		for _, p := range ends {
			for _, c := range place(p, at[2]) {
				reach := vertexReach(at, c)
				if reach > 1e-6 {
					continue
				}
				met++
				require.LessOrEqualf(t, reach, bound,
					`vertex %v publishes bound %.3e but sits %.3e mm from a denoted end`, at, bound, reach)
			}
		}
		require.NotZerof(t, met, `vertex %v meets no denoted segment end`, at)
		if met >= 2 {
			shared++
		}
	}
	return bounded, shared
}

// recordOf records one sketch profile.
func recordOf(t *testing.T, s *sketch.Sketch, p *sketch.Profile) *decad.ProfileRecord {
	t.Helper()
	record, _, err := decad.RecordProfile(s, p)
	require.NoError(t, err)
	return &record
}

// TestExtrudeVertexReachesDenotedCutEnds cuts a radius-7.3 circle centred at
// (0.1, 0.2) with a line along v = 0.3 of length 1370 mm and of length 6e5 mm.
// Each cap meets a line fragment and a circle fragment at two irrational
// crossings, where the line's lerp and the circle's point at their own
// recorded t differ by about ulp(t)·|entity|. The cap reaching u = 7.4 also
// holds the circle's own seam, where its two fragments meet at t = 1 and t = 0
// on the float sum 0.1 + 7.3, which rounds. The extruded and patched caps
// must publish a bound at every such vertex that reaches every denoted end
// meeting there.
func TestExtrudeVertexReachesDenotedCutEnds(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		from, to float64
	}{
		{name: `1370mm line`, from: -370, to: 1000},
		{name: `6e5mm line`, from: -200000, to: 400000},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			s.CreateLine(s.CreatePoint(tc.from, 0.3), s.CreatePoint(tc.to, 0.3))
			s.CreateCircle(s.CreatePoint(0.1, 0.2), 7.3)
			profiles := s.Profiles()
			require.Len(t, profiles, 2)
			doc := decad.New()
			for _, profile := range profiles {
				ends := denotedEnds(t, recordOf(t, s, profile))
				body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
				require.NoError(t, err)
				bounded, shared := requireVerticesReachDenoted(t, body, ends, inPlane)
				junctions := len(ends) / 2
				require.Equal(t, 2*junctions, bounded, `every junction publishes a bound on both caps`)
				require.Equal(t, 2*junctions, shared, `every junction vertex meets both neighbours' ends`)

				patch, err := doc.Patch(t.Context(), s, profile)
				require.NoError(t, err)
				bounded, _ = requireVerticesReachDenoted(t, patch, ends, inPlane)
				require.Equal(t, junctions, bounded, `every patch junction publishes a bound`)
			}
		})
	}
}

// TestExtrudeVertexReachesDenotedCircleSeam extrudes a whole circle. Its one
// seam vertex sits at the walk's float centre + r, which rounds for a centre
// of 0.1 and a radius of 7.3, and must reach the point the circle denotes at
// t = 0 (= t = 1). A circle about an integer centre with an integer radius
// places its seam exactly and stays Exact.
func TestExtrudeVertexReachesDenotedCircleSeam(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		cu, cv  float64
		r       float64
		bounded int
	}{
		{name: `rounded seam`, cu: 0.1, cv: 0.2, r: 7.3, bounded: 2},
		{name: `integer seam`, cu: 3, cv: -2, r: 5, bounded: 0},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			s.CreateCircle(s.CreatePoint(tc.cu, tc.cv), tc.r)
			profiles := s.Profiles()
			require.Len(t, profiles, 1)
			ends := denotedEnds(t, recordOf(t, s, profiles[0]))
			body, err := decad.New().Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
			require.NoError(t, err)
			bounded, _ := requireVerticesReachDenoted(t, body, ends, inPlane)
			require.Equal(t, tc.bounded, bounded)
		})
	}
}

// TestExtrudeVertexReachesDenotedTrimmedSide draws the 10 mm square whose
// right side is a line from (10, −10⁶) cut at the base corner, so that
// fragment's recorded start denotes a point about 1e-10 mm from (10, 0), where
// the base side's natural end sits. The corner vertex must reach both. The
// integer rectangle drawn from four natural sides keeps every vertex Exact,
// as does its integer circular hole.
func TestExtrudeVertexReachesDenotedTrimmedSide(t *testing.T) {
	t.Parallel()
	t.Run(`trimmed side`, func(t *testing.T) {
		t.Parallel()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		origin := s.CreatePoint(0, 0)
		base := s.CreatePoint(10, 0)
		top := s.CreatePoint(10, 10)
		left := s.CreatePoint(0, 10)
		s.CreateLine(origin, base)
		s.CreateLine(s.CreatePoint(10, -1e6), top)
		s.CreateLine(top, left)
		s.CreateLine(left, origin)
		profiles := s.Profiles()
		require.Len(t, profiles, 1)
		ends := denotedEnds(t, recordOf(t, s, profiles[0]))
		body, err := decad.New().Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
		require.NoError(t, err)
		bounded, _ := requireVerticesReachDenoted(t, body, ends, inPlane)
		require.Equal(t, 2, bounded, `only the trimmed corner's two vertices carry a bound`)
	})
	t.Run(`integer rectangle`, func(t *testing.T) {
		t.Parallel()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		a, b := s.CreatePoint(-3, -2), s.CreatePoint(7, -2)
		c, d := s.CreatePoint(7, 4), s.CreatePoint(-3, 4)
		s.CreateLine(a, b)
		s.CreateLine(b, c)
		s.CreateLine(c, d)
		s.CreateLine(d, a)
		profiles := s.Profiles()
		require.Len(t, profiles, 1)
		ends := denotedEnds(t, recordOf(t, s, profiles[0]))
		body, err := decad.New().Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
		require.NoError(t, err)
		bounded, shared := requireVerticesReachDenoted(t, body, ends, inPlane)
		require.Zero(t, bounded, `every vertex of an integer rectangle is Exact`)
		require.Equal(t, 8, shared)
	})
	t.Run(`integer rectangle with a hole`, func(t *testing.T) {
		t.Parallel()
		// The hole loop walks its circle clockwise, from t = 1 to t = 0.
		s, profile := rectWithHoleSketch(t)
		record := recordOf(t, s, profile)
		require.Len(t, record.Holes, 1)
		hole, ok := record.Holes[0].Segments[0].(decad.CircleSeg)
		require.True(t, ok)
		require.Equal(t, [2]float64{1, 0}, [2]float64{hole.TStart, hole.TEnd})
		ends := denotedEnds(t, record)
		body, err := decad.New().Extrude(s, profile, decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
		require.NoError(t, err)
		bounded, _ := requireVerticesReachDenoted(t, body, ends, inPlane)
		require.Zero(t, bounded, `the hole's seam sits on its exact t = 0 point`)
		require.Len(t, body.Vertices(), 10)
	})
}

// TestRevolveVertexReachesDenotedCutEnds revolves the two caps of the 1370 mm
// line cut by the radius-7.3 circle a quarter turn about the line v = 20. A
// junction vertex sits in the profile plane at φ = 0 and at its quarter-turn
// image (u, 20, ±(v − 20)), and must reach both neighbours' denoted ends at
// each.
func TestRevolveVertexReachesDenotedCutEnds(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateLine(s.CreatePoint(-370, 0.3), s.CreatePoint(1000, 0.3))
	s.CreateCircle(s.CreatePoint(0.1, 0.2), 7.3)
	profiles := s.Profiles()
	require.Len(t, profiles, 2)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 20}, End: decad.Point2{U: 1, V: 20}}
	quarter := func(p denotedPoint, _ float64) [][3]*big.Float {
		off := new(big.Float).SetPrec(junctionPrec).Sub(p.v, jf(20))
		neg := new(big.Float).SetPrec(junctionPrec).Neg(off)
		return [][3]*big.Float{{p.u, p.v, jf(0)}, {p.u, jf(20), off}, {p.u, jf(20), neg}}
	}
	doc := decad.New()
	for _, profile := range profiles {
		ends := denotedEnds(t, recordOf(t, s, profile))
		body, err := doc.Revolve(s, profile, axis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		require.NoError(t, err)
		bounded, shared := requireVerticesReachDenoted(t, body, ends, quarter)
		junctions := len(ends) / 2
		require.Equal(t, 2*junctions, bounded, `every junction publishes a bound at both sweep ends`)
		require.Equal(t, 2*junctions, shared)
	}
}

// TestExtrudeVertexReachesDenotedMergedRun draws the 10 mm square's base as
// two collinear lines, (0, 0)–(5, 0) and a long line from (5, 0) to (10⁶, 0)
// that the right side cuts at (10, 0). The build merges the two into one wall,
// whose end is the long line's trimmed end. The corner vertex must reach the
// point that trimmed end denotes, not the bound of the junction the merge
// dropped.
func TestExtrudeVertexReachesDenotedMergedRun(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	mid := s.CreatePoint(5, 0)
	base := s.CreatePoint(10, 0)
	top := s.CreatePoint(10, 10)
	left := s.CreatePoint(0, 10)
	s.CreateLine(origin, mid)
	s.CreateLine(mid, s.CreatePoint(1e6, 0))
	s.CreateLine(base, top)
	s.CreateLine(top, left)
	s.CreateLine(left, origin)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	record := recordOf(t, s, profiles[0])
	ends := denotedEnds(t, record)
	require.Len(t, record.Outer.Segments, 5, `the base stays two recorded segments`)
	body, err := decad.New().Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
	require.NoError(t, err)
	bounded, shared := requireVerticesReachDenoted(t, body, ends, inPlane)
	require.Equal(t, 2, bounded, `only the trimmed corner's two vertices carry a bound`)
	require.Equal(t, 8, shared, `every corner meets both neighbours' ends`)
}

// TestChainVertexReachesDenotedCutEnd records the two open fragments the
// radius-7.3 circle leaves of the 1370 mm line along v = 0.3. Each is one line
// segment trimmed at a crossing, so its free end there is a lerp at the
// recorded t. Every vertex ExtrudeChain and a quarter-turn RevolveChain about
// v = 20 place there must reach the denoted point, and the extruded line's
// natural end stays Exact. The quarter-turn image of every vertex also
// carries the rotation's own rounding, so the revolve's natural end is
// bounded there too.
//
// Shown-to-fail on amd64: with no walk-end term at a chain's free ends,
// ExtrudeChain publishes the trimmed end Exact, 1.8e-15 mm from its denoted
// point. The revolve leg is coverage only: RevolveChain charged its free
// ends' own walk bound already.
func TestChainVertexReachesDenotedCutEnd(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateLine(s.CreatePoint(-370, 0.3), s.CreatePoint(1000, 0.3))
	s.CreateCircle(s.CreatePoint(0.1, 0.2), 7.3)
	chains := s.Chains()
	require.Len(t, chains, 2)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 20}, End: decad.Point2{U: 1, V: 20}}
	quarter := func(p denotedPoint, _ float64) [][3]*big.Float {
		off := new(big.Float).SetPrec(junctionPrec).Sub(p.v, jf(20))
		neg := new(big.Float).SetPrec(junctionPrec).Neg(off)
		return [][3]*big.Float{{p.u, p.v, jf(0)}, {p.u, jf(20), off}, {p.u, jf(20), neg}}
	}
	doc := decad.New()
	for _, ch := range chains {
		record, _, err := decad.RecordChain(s, ch)
		require.NoError(t, err)
		require.Len(t, record.Segments, 1)
		ends := denotedSegmentEnds(t, record.Segments)

		body, err := doc.ExtrudeChain(s, ch, decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
		require.NoError(t, err)
		// Whether the trimmed end's held lerp rounds is arch-specific: an FMA
		// evaluation lands on this lerp exactly, a two-step one does not. Only
		// the natural end is Exact on every arch.
		bounded, _ := requireVerticesReachDenoted(t, body, ends, inPlane)
		require.LessOrEqual(t, bounded, 2, `the natural end's two vertices stay Exact`)

		body, err = doc.RevolveChain(s, ch, axis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		require.NoError(t, err)
		bounded, _ = requireVerticesReachDenoted(t, body, ends, quarter)
		require.GreaterOrEqual(t, bounded, 2, `the trimmed end's vertex at each sweep end carries a bound`)
	}
}
