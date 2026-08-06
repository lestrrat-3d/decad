package decad_test

import (
	"context"
	"encoding/json"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
	"github.com/stretchr/testify/require"
)

// This file is docs/loft-design.md PR 1b: the public-surface tests over
// loft_build_internal_test.go's already-covered evaluator (S1-S8, Table P,
// Table B, §8's mass kernel). Every test here needs a live sketch, the
// public entry point, or the recorded step — nothing PR 1a's internal tests
// already assert is repeated.

// loftSquaresAt builds two sketches on parallel planes at the given world
// origin, each with a centred square rectangle: bottomHalf on the plane
// through origin (normal +Z), topHalf on the plane offset by height along
// that normal. CreateOffsetPlane keeps the same U/V basis, so the natural
// (offset-0) correspondence pairs corresponding corners directly.
func loftSquaresAt(t *testing.T, origin r3.Vec, bottomHalf, topHalf, height float64) (*sketch.Sketch, *sketch.Profile, *sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	base, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	top, err := w.CreateOffsetPlane(base, height)
	require.NoError(t, err)

	s0, err := w.CreateSketch(base)
	require.NoError(t, err)
	r0 := s0.CreateRectangle(-bottomHalf, -bottomHalf, bottomHalf, bottomHalf)
	s0.Fix(r0.A)
	_, err = s0.Solve(t.Context())
	require.NoError(t, err)

	s1, err := w.CreateSketch(top)
	require.NoError(t, err)
	r1 := s1.CreateRectangle(-topHalf, -topHalf, topHalf, topHalf)
	s1.Fix(r1.A)
	_, err = s1.Solve(t.Context())
	require.NoError(t, err)

	return s0, s0.Profiles()[0], s1, s1.Profiles()[0]
}

// loftSquares is loftSquaresAt at the world origin.
func loftSquares(t *testing.T, bottomHalf, topHalf float64) (*sketch.Sketch, *sketch.Profile, *sketch.Sketch, *sketch.Profile) {
	t.Helper()
	return loftSquaresAt(t, r3.NewVec(0, 0, 0), bottomHalf, topHalf, 10)
}

// loftCircleProfile builds a solved circle sketch of radius r on plane, owned by
// w, and returns its single profile.
func loftCircleProfile(t *testing.T, w *sketch.World, plane *sketch.Plane, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// --- Mass properties (§8) ---

func TestLoftBuildsCongruentSquares(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.True(t, vol.Value.Equal(units.CubicMillimeters(16000), 1e-9), "40x40x10 box = 16000 mm^3, got %s", vol.Value)

	bounds, err := body.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, bounds.Exactness)
	require.InDelta(t, -20, bounds.Min.X, 1e-9)
	require.InDelta(t, -20, bounds.Min.Y, 1e-9)
	require.InDelta(t, 0, bounds.Min.Z, 1e-9)
	require.InDelta(t, 20, bounds.Max.X, 1e-9)
	require.InDelta(t, 20, bounds.Max.Y, 1e-9)
	require.InDelta(t, 10, bounds.Max.Z, 1e-9)

	c, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 0, c.Value.X, 1e-9)
	require.InDelta(t, 0, c.Value.Y, 1e-9)
	require.InDelta(t, 5, c.Value.Z, 1e-9)

	faces := body.Faces()
	require.Len(t, faces, 10, "8 wall triangles + capStart + capEnd")
	edges := body.Edges()
	require.Len(t, edges, 16, "4 bottom rim + 4 top rim + 4 diagonal + 4 rung")
	requireManifold(t, body)
	require.Len(t, body.Vertices(), 8)
}

func TestLoftFrustumVolumeMatchesClosedForm(t *testing.T) {
	// bottom side 40 (half 20), top side 20 (half 10), height 10: the
	// pyramidal frustum V = (h/3)(A0 + A1 + sqrt(A0*A1)) = 28000/3 mm^3, not
	// representable in binary, so Volume must be Approximate with a positive
	// proven bound that encloses the closed-form value.
	s0, p0, s1, p1 := loftSquares(t, 20, 10)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	const a0, a1, h = 1600.0, 400.0, 10.0
	wantVol := h / 3 * (a0 + a1 + math.Sqrt(a0*a1))

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	boundVol, err := vol.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Greater(t, boundVol, 0.0)
	require.LessOrEqual(t, math.Abs(gotVol-wantVol), boundVol, "the proven bound must enclose the closed-form frustum volume")

	wantCZ := h * (a0 + 2*math.Sqrt(a0*a1) + 3*a1) / (4 * (a0 + a1 + math.Sqrt(a0*a1)))
	c, err := body.Centroid()
	require.NoError(t, err)
	boundC, err := c.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(c.Value.Z-wantCZ), boundC, "the proven bound must enclose the closed-form frustum centroid")

	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	boundArea, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Greater(t, boundArea, 0.0)
}

// TestLoftAlignmentSelectsTheCorrespondence uses an asymmetric (scalene)
// pentagon so no offset other than the true one can coincidentally reproduce
// the closed-form untwisted volume through a rotational or reflective
// symmetry of the shape itself. p0 records its five corners a,b,c,d,e in that
// order; p1 is the identical footprint (CreateOffsetPlane keeps the same U/V
// basis, so no plane rotation is involved), but its own points are created
// starting from b — b,c,d,e,a — so p1's own recorded segment 0 starts at
// world position b, one step ahead of p0's own segment 0 (at a). Vertex k of
// p0 sits at the SAME world position as p1's vertex (k+4) mod 5 (verified by
// hand and pinned by this test's own offset-4 assertions below), so
// WithLoftAlignment(4) is the one offset that undoes the shift and recovers
// the plain (untwisted) pentagonal prism.
func TestLoftAlignmentSelectsTheCorrespondence(t *testing.T) {
	const height = 10.0
	pts := [5][2]float64{{0, 0}, {10, 0}, {14, 5}, {9, 11}, {1, 8}}
	// Shoelace area of the pentagon above, positive (CCW): 110 mm^2.
	const wantArea = 110.0
	const wantVol = wantArea * height

	w := sketch.NewWorld()
	s0, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p0pts := make([]*sketch.Point, 5)
	for i, p := range pts {
		p0pts[i] = s0.CreatePoint(p[0], p[1])
	}
	for i := range 5 {
		s0.CreateLine(p0pts[i], p0pts[(i+1)%5])
	}
	for _, p := range p0pts {
		s0.Fix(p)
	}
	_, err = s0.Solve(t.Context())
	require.NoError(t, err)
	p0 := s0.Profiles()[0]

	top, err := w.CreateOffsetPlane(w.XY(), height)
	require.NoError(t, err)
	s1, err := w.CreateSketch(top)
	require.NoError(t, err)
	shifted := [5][2]float64{pts[1], pts[2], pts[3], pts[4], pts[0]}
	p1pts := make([]*sketch.Point, 5)
	for i, p := range shifted {
		p1pts[i] = s1.CreatePoint(p[0], p[1])
	}
	for i := range 5 {
		s1.CreateLine(p1pts[i], p1pts[(i+1)%5])
	}
	for _, p := range p1pts {
		s1.Fix(p)
	}
	_, err = s1.Solve(t.Context())
	require.NoError(t, err)
	p1 := s1.Profiles()[0]

	// countRungs reports how many edges of body run z-parallel (identical
	// x, y, differing z) and asserts every one of them spans the full sweep
	// height — the "vertical rung" signature of an untwisted correspondence.
	countRungs := func(t *testing.T, body *decad.Body) int {
		t.Helper()
		rungs := 0
		for _, e := range body.Edges() {
			a := e.Start().Position().Value
			b := e.End().Position().Value
			if math.Abs(a.X-b.X) > 1e-9 || math.Abs(a.Y-b.Y) > 1e-9 {
				continue
			}
			require.InDelta(t, height, math.Abs(a.Z-b.Z), 1e-9, "a z-parallel edge must span the full sweep height")
			rungs++
		}
		return rungs
	}

	t.Run("DefaultOffsetIsTheWrongCorrespondence", func(t *testing.T) {
		// Omitting WithLoftAlignment (every offset 0) does not undo the
		// one-step shift, so it builds a twisted, non-vertical-rung pentagon
		// loft whose volume is strictly less than the untwisted one.
		doc := decad.New()
		body, err := doc.Loft(s0, p0, s1, p1)
		require.NoError(t, err)
		vol, err := body.Volume()
		require.NoError(t, err)
		gotVol, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.Less(t, gotVol, wantVol)
		require.Equal(t, 0, countRungs(t, body), "the default (unaligned) correspondence has no vertical rung")
	})

	t.Run("OffsetFourReachesTheUntwistedCorrespondence", func(t *testing.T) {
		doc := decad.New()
		body, err := doc.Loft(s0, p0, s1, p1, decad.WithLoftAlignment(4))
		require.NoError(t, err)
		vol, err := body.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, vol.Exactness)
		gotVol, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.InDelta(t, wantVol, gotVol, 1e-6)
		require.Equal(t, 5, countRungs(t, body), "every vertex must have exactly one z-parallel rung")
	})

	t.Run("EveryOffsetEitherBuildsAtOrBelowTheUntwistedVolumeOrRefuses", func(t *testing.T) {
		for off := range 5 {
			doc := decad.New()
			body, err := doc.Loft(s0, p0, s1, p1, decad.WithLoftAlignment(off))
			if err != nil {
				require.ErrorIsf(t, err, decad.ErrDegenerate, "offset %d must either build or refuse with S7's ErrDegenerate", off)
				continue
			}
			vol, err := body.Volume()
			require.NoError(t, err)
			gotVol, err := vol.Value.In(units.CubicMillimeter)
			require.NoError(t, err)
			require.LessOrEqualf(t, gotVol, wantVol+1e-6, "offset %d: no correspondence may exceed the untwisted volume", off)
		}
	})
}

// --- Seam gates (S9) ---

func TestLoftSeamGates(t *testing.T) {
	t.Run("ForeignProfileAtP0", func(t *testing.T) {
		s0, _, s1, p1 := loftSquares(t, 20, 20)
		w := sketch.NewWorld()
		foreign, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		fr := foreign.CreateRectangle(-20, -20, 20, 20)
		foreign.Fix(fr.A)
		_, err = foreign.Solve(t.Context())
		require.NoError(t, err)

		doc := decad.New()
		body, err := doc.Loft(s0, foreign.Profiles()[0], s1, p1)
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrForeignProfile)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})

	t.Run("ForeignProfileAtP1", func(t *testing.T) {
		s0, p0, s1, _ := loftSquares(t, 20, 20)
		w := sketch.NewWorld()
		foreign, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		fr := foreign.CreateRectangle(-20, -20, 20, 20)
		foreign.Fix(fr.A)
		_, err = foreign.Solve(t.Context())
		require.NoError(t, err)

		doc := decad.New()
		body, err := doc.Loft(s0, p0, s1, foreign.Profiles()[0])
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrForeignProfile)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})

	t.Run("StaleProfileAtP0", func(t *testing.T) {
		s0, p0, s1, p1 := loftSquares(t, 20, 20)
		s0.AddConstraint(sketch.NewDistance(s0.Points()[0], s0.Points()[1], 55))
		_, err := s0.Solve(t.Context())
		require.NoError(t, err)
		require.True(t, p0.IsStale())

		doc := decad.New()
		body, err := doc.Loft(s0, p0, s1, p1)
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrStaleProfile)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})

	t.Run("StaleProfileAtP1", func(t *testing.T) {
		s0, p0, s1, p1 := loftSquares(t, 20, 20)
		s1.AddConstraint(sketch.NewDistance(s1.Points()[0], s1.Points()[1], 55))
		_, err := s1.Solve(t.Context())
		require.NoError(t, err)
		require.True(t, p1.IsStale())

		doc := decad.New()
		body, err := doc.Loft(s0, p0, s1, p1)
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrStaleProfile)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})

	t.Run("InvalidProfileAtP0", func(t *testing.T) {
		w := sketch.NewWorld()
		s0, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		first := s0.CreateRectangle(0, 0, 10, 10)
		second := s0.CreateRectangle(20, 0, 30, 10)
		s0.Fix(first.A)
		s0.Fix(second.A)
		_, err = s0.Solve(t.Context())
		require.NoError(t, err)
		profiles := s0.Profiles()
		require.Len(t, profiles, 2)
		profiles[0].Outer = profiles[1].Outer // a caller-altered snapshot

		top, err := w.CreateOffsetPlane(w.XY(), 10)
		require.NoError(t, err)
		s1, err := w.CreateSketch(top)
		require.NoError(t, err)
		r1 := s1.CreateRectangle(-5, -5, 5, 5)
		s1.Fix(r1.A)
		_, err = s1.Solve(t.Context())
		require.NoError(t, err)

		doc := decad.New()
		body, err := doc.Loft(s0, profiles[0], s1, s1.Profiles()[0])
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrInvalidProfile)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})

	t.Run("InvalidProfileAtP1", func(t *testing.T) {
		w := sketch.NewWorld()
		base, err := w.CreateOffsetPlane(w.XY(), 10)
		require.NoError(t, err)
		s1, err := w.CreateSketch(base)
		require.NoError(t, err)
		first := s1.CreateRectangle(0, 0, 10, 10)
		second := s1.CreateRectangle(20, 0, 30, 10)
		s1.Fix(first.A)
		s1.Fix(second.A)
		_, err = s1.Solve(t.Context())
		require.NoError(t, err)
		profiles := s1.Profiles()
		require.Len(t, profiles, 2)
		profiles[0].Outer = profiles[1].Outer

		s0, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		r0 := s0.CreateRectangle(-5, -5, 5, 5)
		s0.Fix(r0.A)
		_, err = s0.Solve(t.Context())
		require.NoError(t, err)

		doc := decad.New()
		body, err := doc.Loft(s0, s0.Profiles()[0], s1, profiles[0])
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrInvalidProfile)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})
}

// --- Pre-gates: nil arguments (S10), options (S11, S4 arity) ---

func TestLoftNilArguments(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	cases := []struct {
		name string
		s0   *sketch.Sketch
		p0   *sketch.Profile
		s1   *sketch.Sketch
		p1   *sketch.Profile
	}{
		{"s0", nil, p0, s1, p1},
		{"p0", s0, nil, s1, p1},
		{"s1", s0, p0, nil, p1},
		{"p1", s0, p0, s1, nil},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			body, err := doc.Loft(tc.s0, tc.p0, tc.s1, tc.p1)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Empty(t, doc.Bodies())
			require.Empty(t, doc.Recipe().Steps)
		})
	}
}

// unknownLoftOption is a foreign LoftOption implementation: it embeds the
// interface to promote its sealed marker, then overrides Ident() with a
// counter so a test can prove Loft never invokes it on a rejected option.
type unknownLoftOption struct {
	decad.LoftOption
	calls *int
}

type unknownLoftOptionIdent struct{}

func (o unknownLoftOption) Ident() any {
	(*o.calls)++
	return unknownLoftOptionIdent{}
}

func TestLoftForeignOption(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)

	t.Run("ForeignImplementation", func(t *testing.T) {
		doc := decad.New()
		calls := 0
		opt := unknownLoftOption{
			LoftOption: decad.WithLoftAlignment(0),
			calls:      &calls,
		}
		body, err := doc.Loft(s0, p0, s1, p1, opt)
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "not a decad loft option")
		require.Zero(t, calls, "Loft rejects a foreign option before invoking its callback")
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})

	t.Run("NilElement", func(t *testing.T) {
		doc := decad.New()
		body, err := doc.Loft(s0, p0, s1, p1, nil)
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})
}

func TestLoftDuplicateAlignmentOption(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1, decad.WithLoftAlignment(0), decad.WithLoftAlignment(0))
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())
	require.Empty(t, doc.Recipe().Steps)
}

func TestLoftEmptyAlignmentPayload(t *testing.T) {
	// A one-loop pair needs exactly one offset; WithLoftAlignment() with no
	// arguments must reach validateLoftRecords as a non-nil, length-0 slice
	// so it refuses as S4 (wrong length) rather than silently defaulting.
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1, decad.WithLoftAlignment())
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())
	require.Empty(t, doc.Recipe().Steps)
}

// --- Shape gates (S3, S5) ---

func TestLoftCoplanarSectionsRefuse(t *testing.T) {
	t.Run("SameSketch", func(t *testing.T) {
		s0, p0, _, _ := loftSquares(t, 20, 20)
		doc := decad.New()
		body, err := doc.Loft(s0, p0, s0, p0)
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})

	t.Run("RotatedBasisSamePlane", func(t *testing.T) {
		w := sketch.NewWorld()
		s0, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		r0 := s0.CreateRectangle(-20, -20, 20, 20)
		s0.Fix(r0.A)
		_, err = s0.Solve(t.Context())
		require.NoError(t, err)

		frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0))
		require.NoError(t, err)
		rotPlane, err := w.CreatePlaneFromFrame(frame)
		require.NoError(t, err)
		s1, err := w.CreateSketch(rotPlane)
		require.NoError(t, err)
		r1 := s1.CreateRectangle(-20, -20, 20, 20)
		s1.Fix(r1.A)
		_, err = s1.Solve(t.Context())
		require.NoError(t, err)

		doc := decad.New()
		body, err := doc.Loft(s0, s0.Profiles()[0], s1, s1.Profiles()[0])
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})
}

func TestLoftCurvedPairRefuses(t *testing.T) {
	// Both scenarios refuse (whichever of S2/S3 the evaluator reaches first
	// depends on the recorded segment counts; the public entry point's own
	// job is to propagate the evaluator's refusal, already pinned exactly by
	// loft_build_internal_test.go's own S1-S8 tests).
	t.Run("CircleAgainstSquare", func(t *testing.T) {
		w := sketch.NewWorld()
		s0, p0 := loftCircleProfile(t, w, w.XY(), 10)
		top, err := w.CreateOffsetPlane(w.XY(), 10)
		require.NoError(t, err)
		s1, err := w.CreateSketch(top)
		require.NoError(t, err)
		r1 := s1.CreateRectangle(-10, -10, 10, 10)
		s1.Fix(r1.A)
		_, err = s1.Solve(t.Context())
		require.NoError(t, err)

		doc := decad.New()
		body, err := doc.Loft(s0, p0, s1, s1.Profiles()[0])
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})

	t.Run("CircleAgainstCircle", func(t *testing.T) {
		// Matching segment counts (one CircleSeg each), so this is squarely
		// S3: a same-kind curved pair, refused even where the two loops
		// otherwise pair one-to-one.
		w := sketch.NewWorld()
		s0, p0 := loftCircleProfile(t, w, w.XY(), 10)
		top, err := w.CreateOffsetPlane(w.XY(), 10)
		require.NoError(t, err)
		s1, p1 := loftCircleProfile(t, w, top, 5)

		doc := decad.New()
		body, err := doc.Loft(s0, p0, s1, p1)
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})
}

// --- Recipe fidelity ---

func TestLoftRecordsStep(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1, decad.WithLoftAlignment(0))
	require.NoError(t, err)
	require.NotNil(t, body)

	wantProfile0, wantPlane0, err := decad.RecordProfile(s0, p0)
	require.NoError(t, err)
	wantProfile1, wantPlane1, err := decad.RecordProfile(s1, p1)
	require.NoError(t, err)

	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 1)
	step := recipe.Steps[0]
	require.Equal(t, decad.OpLoft, step.Op)
	require.Empty(t, step.Inputs)
	require.Nil(t, step.Extent)
	require.Nil(t, step.Angular)
	require.Nil(t, step.Axis)
	require.Empty(t, step.Selectors)
	require.Empty(t, step.Values)
	require.Equal(t, decad.TransformRecord{}, step.Placement)
	require.Equal(t, wantProfile0, step.Profile)
	require.Equal(t, wantPlane0, step.Plane)

	opts, ok := step.Opts.(decad.LoftOpts)
	require.True(t, ok)
	require.Equal(t, wantProfile1, opts.Profile2)
	require.Equal(t, wantPlane1, opts.Plane2)
	require.Equal(t, []int{0}, opts.Alignment)
}

func TestLoftRecordsOmittedAlignmentAsNil(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	_, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	step := doc.Recipe().Steps[0]
	opts, ok := step.Opts.(decad.LoftOpts)
	require.True(t, ok)
	require.Nil(t, opts.Alignment, "an omitted WithLoftAlignment records no offsets, never an explicit all-zero list")
}

func TestLoftRecipeDoesNotAliasTheDocument(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	offsets := []int{0}
	_, err := doc.Loft(s0, p0, s1, p1, decad.WithLoftAlignment(offsets...))
	require.NoError(t, err)

	// (a) mutating the caller's own slice after the call must not reach the
	// document — WithLoftAlignment copies it at the call.
	offsets[0] = 99
	recipe := doc.Recipe()
	opts := recipe.Steps[0].Opts.(decad.LoftOpts)
	require.Equal(t, []int{0}, opts.Alignment)

	// (b) mutating a value handed out by Recipe() must not reach a second
	// Recipe() call — cloneStepOpts' fix.
	opts.Alignment[0] = 42
	opts.Profile2.Outer.Segments[0] = decad.LineSeg{
		Start: decad.Point2{U: 999, V: 999}, End: decad.Point2{U: 999, V: 999}, TStart: 0, TEnd: 1,
	}

	recipe2 := doc.Recipe()
	opts2 := recipe2.Steps[0].Opts.(decad.LoftOpts)
	require.Equal(t, []int{0}, opts2.Alignment, "the document's own recorded alignment must not alias a caller-visible slice")
	seg, ok := opts2.Profile2.Outer.Segments[0].(decad.LineSeg)
	require.True(t, ok)
	require.NotEqual(t, decad.Point2{U: 999, V: 999}, seg.Start, "the document's own recorded section must not alias a caller-visible slice")
}

func TestLoftOptionAliasDoesNotReachTheDocument(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	opt := decad.WithLoftAlignment(0)
	_, err := doc.Loft(s0, p0, s1, p1, opt)
	require.NoError(t, err)

	// option.Get is a plain type assertion (option/v3's Get: "v, ok :=
	// opt.value().(T)") — it hands back the option's own stored slice
	// header, not a copy. A caller who keeps the LoftOption and reads its
	// payload back this way (or through Option[[]int].Value()) must not be
	// able to reach the document's own recorded step through it.
	v, ok := option.Get[[]int](opt)
	require.True(t, ok)
	v[0] = 3

	recipe := doc.Recipe()
	opts := recipe.Steps[0].Opts.(decad.LoftOpts)
	require.Equal(t, []int{0}, opts.Alignment, "the recorded step must not alias the caller's retained option payload")
}

func TestLoftSharedOptionAcrossCallsDoesNotAliasSteps(t *testing.T) {
	s0a, p0a, s1a, p1a := loftSquares(t, 20, 20)
	s0b, p0b, s1b, p1b := loftSquares(t, 20, 20)
	doc := decad.New()
	shared := decad.WithLoftAlignment(0)

	_, err := doc.Loft(s0a, p0a, s1a, p1a, shared)
	require.NoError(t, err)
	_, err = doc.Loft(s0b, p0b, s1b, p1b, shared)
	require.NoError(t, err)

	v, ok := option.Get[[]int](shared)
	require.True(t, ok)
	v[0] = 7

	recipe := doc.Recipe()
	opts0 := recipe.Steps[0].Opts.(decad.LoftOpts)
	opts1 := recipe.Steps[1].Opts.(decad.LoftOpts)
	require.Equal(t, []int{0}, opts0.Alignment, "one shared LoftOption's payload must not alias the first recorded step")
	require.Equal(t, []int{0}, opts1.Alignment, "one shared LoftOption's payload must not alias the second recorded step")
}

func TestLoftRecipeRoundTrip(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	_, err := doc.Loft(s0, p0, s1, p1, decad.WithLoftAlignment(0))
	require.NoError(t, err)

	recipe := doc.Recipe()
	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, "the recorded recipe, including a non-zero Alignment, round-trips exactly")
}

// --- Verify wiring (D6) ---

func TestLoftVerifySound(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 10)
	doc := decad.New()
	_, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	require.Empty(t, report.Diagnostics)
	require.Len(t, report.Bodies, 1)

	br := report.Bodies[0]
	require.True(t, br.Solid)
	require.True(t, br.Watertight)
	require.True(t, br.Manifold)
	require.False(t, br.SelfIntersecting)
	require.Equal(t, 1, br.Lumps)
	require.Equal(t, 0, br.Voids)
	require.NotNil(t, br.Volume)
	require.NotNil(t, br.Centroid)
}

func TestLoftVerifySurveysStaySuspect(t *testing.T) {
	cases := []struct {
		name string
		opt  decad.VerifyOption
		code decad.DiagnosticCode
	}{
		{"wall", decad.WithMinWallThickness(units.Millimeters(1)), decad.DiagUndecidedWall},
		{"pull", decad.WithPullDirection(r3.NewVec(0, 0, 1)), decad.DiagUndecidedUndercut},
		{"radius", decad.WithMinRadius(), decad.DiagUndecidedMinRadius},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			s0, p0, s1, p1 := loftSquares(t, 20, 10)
			doc := decad.New()
			_, err := doc.Loft(s0, p0, s1, p1)
			require.NoError(t, err)

			report, err := doc.Verify(t.Context(), tc.opt)
			require.NoError(t, err)
			require.Equal(t, decad.Suspect, report.Status)
			diag, ok := findDiagnostic(report.Diagnostics, tc.code)
			require.True(t, ok, "expected diagnostic %s", tc.code)
			require.Equal(t, decad.Suspect, diag.Status)
		})
	}
}

func TestLoftVerifyBoxDisjointPairIsSound(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 10, 10)
	doc := decad.New()
	_, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	t0, tp0, t1, tp1 := loftSquaresAt(t, r3.NewVec(1000, 0, 0), 10, 10, 10)
	_, err = doc.Loft(t0, tp0, t1, tp1)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Interferences)

	report, err = doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Clearances, "the analytic clearance kernel has no loft case yet (D4)")
	diag, ok := findDiagnostic(report.Diagnostics, decad.DiagUndecidedClearance)
	require.True(t, ok)
	require.Equal(t, decad.Suspect, diag.Status)
}

// --- Staged downstream (D1) ---

func TestLoftTessellateStaged(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 10)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(1))
	require.Nil(t, mesh)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// --- Cancellation ---

func TestLoftContextCancellation(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()

	body, err := doc.LoftContext(ctx, s0, p0, s1, p1)
	require.Nil(t, body)
	require.ErrorIs(t, err, context.Canceled)
	require.Empty(t, doc.Bodies())
	require.Empty(t, doc.Recipe().Steps)
}
