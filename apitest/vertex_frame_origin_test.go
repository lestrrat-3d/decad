package apitest_test

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins the frame-ORIGIN half of every analytic builder's vertex
// lift (docs/evaluator-design.md §8): a sketch plane whose U and V are the
// world X and Y axes still rounds every vertex it lifts when its origin is
// not a number the plane-local coordinate adds to exactly. Each vertex must
// sit within its published bound of the exact rational lift of the point it
// denotes, whatever the payload kind, and a lift that is exact must keep its
// zero bound.

// originShift offsets every plane-local u so that origin.X + u rounds at
// both fixture origins.
const originShift = 0.1

// frameOrigins are the two fixture origins: a near one whose own coordinates
// are not dyadic, and a far one where one ulp is about 1.2e-10.
var frameOrigins = []r3.Vec{
	r3.NewVec(0.1, 0.2, 0.3),
	r3.NewVec(1e6+0.1, 0, 0),
}

// liftCand is one exact plane-local point: the recorded u (u + originShift,
// rounded as the sketch records it) plus an exact offset du, v and z, lifted
// through an XY-parallel frame at o and then the exact translation tr.
type liftCand struct {
	o           r3.Vec
	u, du, v, z float64
	tr          r3.Vec
}

// truth is the exact rational world point c denotes.
func (c liftCand) truth() [3]*big.Rat {
	add := func(xs ...*big.Rat) *big.Rat {
		out := new(big.Rat)
		for _, x := range xs {
			out.Add(out, x)
		}
		return out
	}
	return [3]*big.Rat{
		add(ratOf(c.o.X), ratOf(c.u+originShift), ratOf(c.du), ratOf(c.tr.X)),
		add(ratOf(c.o.Y), ratOf(c.v), ratOf(c.tr.Y)),
		add(ratOf(c.o.Z), ratOf(c.z), ratOf(c.tr.Z)),
	}
}

func liftGrid(o r3.Vec, us, vs, zs []float64) []liftCand {
	var out []liftCand
	for _, u := range us {
		for _, v := range vs {
			for _, z := range zs {
				out = append(out, liftCand{o: o, u: u, v: v, z: z})
			}
		}
	}
	return out
}

func shiftedCands(cands []liftCand, tr r3.Vec) []liftCand {
	out := make([]liftCand, len(cands))
	for i, c := range cands {
		c.tr = tr
		out[i] = c
	}
	return out
}

// originRectSketch draws the rectangle [u0, u1] × [v0, v1], every u shifted by
// originShift, on an XY-parallel sketch plane at o.
func originRectSketch(t *testing.T, o r3.Vec, u0, v0, u1, v1 float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	f, err := r3.NewFrame(o, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(u0+originShift, v0, u1+originShift, v1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

func originBox(t *testing.T, doc *decad.Document, o r3.Vec, u0, v0, u1, v1, h float64, dir decad.Direction) *decad.Body {
	t.Helper()
	s, p := originRectSketch(t, o, u0, v0, u1, v1)
	b, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(h), Dir: dir})
	require.NoError(t, err)
	return b
}

// requireVerticesEnclosed matches every vertex of b to its nearest candidate
// and requires the exact residual to sit inside the vertex's published bound.
// want is how many vertices must match a candidate (every vertex when it is
// len(b.Vertices())). It returns the largest residual, so a caller can prove
// the fixture genuinely rounds.
func requireVerticesEnclosed(t *testing.T, b *decad.Body, cands []liftCand, want int) float64 {
	t.Helper()
	truths := make([][3]*big.Rat, len(cands))
	for i, c := range cands {
		truths[i] = c.truth()
	}
	matched, worst := 0, 0.0
	for _, v := range b.Vertices() {
		pos := v.Position()
		res := math.Inf(1)
		for _, truth := range truths {
			res = math.Min(res, vertexResidual(pos.Value, truth))
		}
		if res > 1e-6 {
			continue
		}
		matched++
		worst = math.Max(worst, res)
		require.LessOrEqualf(t, res, pos.Bound.Base(),
			"vertex %v (%v) sits %g from its exact lift, outside its bound %g", pos.Value, pos.Exactness, res, pos.Bound.Base())
	}
	require.Equal(t, want, matched, "every vertex the fixture names must be found")
	return worst
}

// chamferTop is the four top-face corners of a 1 mm chamfer on the
// [0,4]×[0,4] box of height 2: the recorded u plus or minus 1, exactly.
func chamferTop(o r3.Vec) []liftCand {
	return []liftCand{
		{o: o, u: 0, du: 1, v: 1, z: 2}, {o: o, u: 4, du: -1, v: 1, z: 2},
		{o: o, u: 0, du: 1, v: 3, z: 2}, {o: o, u: 4, du: -1, v: 3, z: 2},
	}
}

// TestVertexFrameOriginBoundEnclosesLift builds every analytic payload kind
// on an XY-parallel sketch plane at a non-dyadic origin and at a far one, and
// requires every vertex to sit within its published bound of the exact
// rational lift of the point it denotes.
//
// Shown to fail: with the lift term dropped from prismPayload.liftedVertex and
// the comparison dropped from revolvePayload.sweptVertex (answering 0, the old
// axis-aligned exemption that ignored the frame origin), every subtest below
// goes red — extrude, revolve, chamfer, pocket at both origins (vertices claim
// Exact while off by up to 4e-16 near and 2.33e-11 far), union and stepped at
// the far origin.
func TestVertexFrameOriginBoundEnclosesLift(t *testing.T) {
	t.Parallel()
	for _, o := range frameOrigins {
		t.Run(fmt.Sprintf("extrude@%v", o), func(t *testing.T) {
			t.Parallel()
			b := originBox(t, decad.New(), o, 0, 0, 1, 1, 1, decad.Along)
			worst := requireVerticesEnclosed(t, b, liftGrid(o, []float64{0, 1}, []float64{0, 1}, []float64{0, 1}), 8)
			require.Positive(t, worst, "the fixture must genuinely round")
		})
		t.Run(fmt.Sprintf("revolve@%v", o), func(t *testing.T) {
			t.Parallel()
			s, p := originRectSketch(t, o, 0, 1, 3, 2)
			b, err := decad.New().Revolve(s, p, decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}},
				decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
			require.NoError(t, err)
			// Only the start-plane vertices (phi = 0) have an angle-free exact lift.
			worst := requireVerticesEnclosed(t, b, liftGrid(o, []float64{0, 3}, []float64{1, 2}, []float64{0}), 4)
			require.Positive(t, worst, "the fixture must genuinely round")
		})
		t.Run(fmt.Sprintf("chamfer@%v", o), func(t *testing.T) {
			t.Parallel()
			box := originBox(t, decad.New(), o, 0, 0, 4, 4, 2, decad.Along)
			c, err := box.Chamfer(t.Context(), decad.Edges(decad.CreatedBy(decad.CapEnd(box))), units.Millimeters(1))
			require.NoError(t, err)
			cands := append(liftGrid(o, []float64{0, 4}, []float64{0, 4}, []float64{0, 1}), chamferTop(o)...)
			worst := requireVerticesEnclosed(t, c, cands, 12)
			require.Positive(t, worst, "the fixture must genuinely round")
		})
		t.Run(fmt.Sprintf("union@%v", o), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			a := originBox(t, doc, o, 0, 0, 1, 1, 1, decad.Along)
			b := originBox(t, doc, o, 0.5, 0, 1.5, 1, 1, decad.Along)
			u, err := decad.Union(t.Context(), a, b)
			require.NoError(t, err)
			requireVerticesEnclosed(t, u, liftGrid(o, []float64{0, 0.5, 1, 1.5}, []float64{0, 1}, []float64{0, 1}), 8)
		})
		t.Run(fmt.Sprintf("pocket@%v", o), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			blk := originBox(t, doc, o, 0, 0, 4, 4, 2, decad.Against)
			tool := originBox(t, doc, o, 1, 1, 3, 3, 1, decad.Against)
			cut, err := decad.Cut(t.Context(), blk, tool)
			require.NoError(t, err)
			worst := requireVerticesEnclosed(t, cut,
				liftGrid(o, []float64{0, 1, 3, 4}, []float64{0, 1, 3, 4}, []float64{0, -1, -2}), 16)
			require.Positive(t, worst, "the fixture must genuinely round")
		})
		t.Run(fmt.Sprintf("stepped@%v", o), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			s1 := originBox(t, doc, o, 0, 0, 2, 1, 1, decad.Along)
			s2 := originBox(t, doc, o, 1, 0, 3, 1, 2, decad.Along)
			st, err := decad.Union(t.Context(), s1, s2)
			require.NoError(t, err)
			requireVerticesEnclosed(t, st, liftGrid(o, []float64{0, 1, 2, 3}, []float64{0, 1}, []float64{0, 1, 2}), 12)
		})
	}
}

// TestVertexFarBuiltPlacedBackBoundEnclosesLift builds a box and a chamfered
// box on a sketch plane 10⁶ mm out and places each back by (−10⁶, 0, 0).
// The vertices end up near the world origin, but they rounded at the far lift,
// and their bounds must cover that.
//
// Shown to fail: with the lift term dropped from prismPayload.liftedVertex
// (the old charge read the PLACED origin's magnitude, about 0.1), the box
// vertices publish a bound near 2.5e-14 against a 2.33e-11 error and both
// subtests go red.
func TestVertexFarBuiltPlacedBackBoundEnclosesLift(t *testing.T) {
	t.Parallel()
	o := r3.NewVec(1e6+0.1, 0, 0)
	tr := r3.NewVec(-1e6, 0, 0)
	back, err := r3.Translation(tr)
	require.NoError(t, err)

	t.Run("box", func(t *testing.T) {
		t.Parallel()
		b := originBox(t, decad.New(), o, 0, 0, 1, 1, 1, decad.Along)
		moved, err := b.Placed(t.Context(), back)
		require.NoError(t, err)
		worst := requireVerticesEnclosed(t, moved,
			shiftedCands(liftGrid(o, []float64{0, 1}, []float64{0, 1}, []float64{0, 1}), tr), 8)
		require.Positive(t, worst, "the fixture must genuinely round")
	})
	t.Run("chamfer", func(t *testing.T) {
		t.Parallel()
		box := originBox(t, decad.New(), o, 0, 0, 4, 4, 2, decad.Along)
		c, err := box.Chamfer(t.Context(), decad.Edges(decad.CreatedBy(decad.CapEnd(box))), units.Millimeters(1))
		require.NoError(t, err)
		moved, err := c.Placed(t.Context(), back)
		require.NoError(t, err)
		cands := append(liftGrid(o, []float64{0, 4}, []float64{0, 4}, []float64{0, 1}), chamferTop(o)...)
		worst := requireVerticesEnclosed(t, moved, shiftedCands(cands, tr), 12)
		require.Positive(t, worst, "the fixture must genuinely round")
	})
}

// TestClearanceFrameOriginGapEnclosesExactGap reads the clearance between two
// boxes drawn on one XY-parallel sketch plane, at the origin and at both
// fixture origins, and requires the published gap's bound to cover the exact
// rational gap between the two faces the records denote.
//
// Shown to fail: with the lift term dropped from the clearance prism carriers
// (clearance.PrismCarrierFrame.point recording no LiftRound), the far origin's
// gap reads Exact 0.39999999990686774 against an exact 0.39999999999999991 and
// the far subtest goes red.
func TestClearanceFrameOriginGapEnclosesExactGap(t *testing.T) {
	t.Parallel()
	const near, far = 0.45, 0.85
	for _, o := range append([]r3.Vec{{}}, frameOrigins...) {
		t.Run(fmt.Sprintf("%v", o), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			originBox(t, doc, o, 0, 0, near, 1, 1, decad.Along)
			originBox(t, doc, o, far, 0, far+1, 1, 1, decad.Along)
			rep, err := doc.Verify(t.Context(), decad.WithClearances())
			require.NoError(t, err)
			require.Len(t, rep.Clearances, 1)
			exact := new(big.Rat).Sub(
				liftCand{o: o, u: far}.truth()[0],
				liftCand{o: o, u: near}.truth()[0],
			)
			gap := rep.Clearances[0].Gap
			res := new(big.Rat).Sub(ratOf(gap.Value.Base()), exact)
			res.Abs(res)
			f, isExact := res.Float64()
			if !isExact {
				f = math.Nextafter(f, math.Inf(1))
			}
			require.LessOrEqualf(t, f, gap.Bound.Base(), "gap %v (%v) sits %g from the exact gap, outside its bound %g",
				gap.Value, gap.Exactness, f, gap.Bound.Base())
		})
	}
}

// TestVertexIntegerOriginBoundStaysZero is the exact half of the per-vertex
// lift charge: on an XY-parallel sketch plane at a nonzero INTEGER origin, an
// integer rectangle lifts exactly, so every prism, revolve and chamfer vertex
// keeps Exact with a zero bound (clearance_lift_internal_test.go pins the
// same for the clearance kernel's carriers). Shown to fail: replacing the
// exact measurement with a magnitude charge (proofbound.RigidRoundAllow at the
// coordinate and origin envelope) in prismPayload.liftedVertex turns the prism
// and chamfer subtests red, and in revolvePayload.sweptVertex the revolve
// subtest.
func TestVertexIntegerOriginBoundStaysZero(t *testing.T) {
	t.Parallel()
	o := r3.NewVec(10, 20, 30)
	intRect := func(t *testing.T, u0, v0, u1, v1 float64) (*sketch.Sketch, *sketch.Profile) {
		t.Helper()
		f, err := r3.NewFrame(o, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		w := sketch.NewWorld()
		plane, err := w.CreatePlaneFromFrame(f)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(u0, v0, u1, v1)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	extrude := func(t *testing.T, doc *decad.Document, u0, v0, u1, v1, h float64) *decad.Body {
		t.Helper()
		s, p := intRect(t, u0, v0, u1, v1)
		b, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
		require.NoError(t, err)
		return b
	}
	requireAllZero := func(t *testing.T, vs []*decad.Vertex) {
		t.Helper()
		require.NotEmpty(t, vs)
		for _, v := range vs {
			pos := v.Position()
			require.Equal(t, decad.Exact, pos.Exactness)
			require.Equal(t, units.Millimeters(0), pos.Bound)
		}
	}

	t.Run("prism", func(t *testing.T) {
		t.Parallel()
		requireAllZero(t, extrude(t, decad.New(), 0, 0, 100, 60, 40).Vertices())
	})
	t.Run("revolve", func(t *testing.T) {
		t.Parallel()
		s, p := intRect(t, 0, 5, 10, 15)
		b, err := decad.New().Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		require.NoError(t, err)
		// The phi = 90° cap reads the libm's own sine and cosine, whose
		// angular term is charged apart; only the start plane is exact.
		var start []*decad.Vertex
		for _, v := range b.Vertices() {
			if v.Position().Value.Z == o.Z {
				start = append(start, v)
			}
		}
		require.Len(t, start, 4)
		requireAllZero(t, start)
	})
	t.Run("chamfer", func(t *testing.T) {
		t.Parallel()
		box := extrude(t, decad.New(), 0, 0, 100, 60, 20)
		c, err := box.Chamfer(t.Context(), decad.Edges(decad.CreatedBy(decad.CapEnd(box))), units.Millimeters(5))
		require.NoError(t, err)
		requireAllZero(t, c.Vertices())
	})
}
