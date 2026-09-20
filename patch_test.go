package decad_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's T2-shaped public-surface tests for
// Document.Patch: a single planar face built directly from a recorded
// profile, on the sketch plane's own frame (§5.1). Every fixture reuses
// plateSketch (extrude_test.go) and rectWithHoleSketch (surface_test.go).

// TestPatchIsAPlanarSheet is docs/surface-design.md's T2 in full: a 100x60 mm
// rectangle patched directly from its sketch.
func TestPatchIsAPlanarSheet(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(s, p)
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, patch.Kind())
	require.False(t, patch.IsSolid())
	_, err = patch.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = patch.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	faces, err := decad.Faces(decad.Planar()).Exactly(1).SelectFaces(patch)
	require.NoError(t, err)
	face := faces[0]

	decadtest.MeasuresArea(t, patch, units.SquareMillimeters(6000), decadtest.Exactly())

	free, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(patch)
	require.NoError(t, err)
	for _, e := range free {
		require.True(t, e.IsFree())
		require.Len(t, e.Faces(), 1)
		require.True(t, e.IsConvex(), "every outer edge of a rectangle patch is convex")
	}

	require.Len(t, patch.Lumps(), 1)
	shells := patch.Shells()
	require.Len(t, shells, 1)
	require.True(t, shells[0].IsOpen())
	require.False(t, shells[0].IsVoid())

	// The positive side is the sketch plane's own normal (§5.1): plateSketch
	// draws on the XY plane, so the outward normal is +Z, Exact.
	n, err := face.NormalAt(r3.NewVec(50, 30, 0))
	require.NoError(t, err)
	require.Equal(t, decad.Exact, n.Exactness)
	require.InDelta(t, 0, n.Value.X, 1e-12)
	require.InDelta(t, 0, n.Value.Y, 1e-12)
	require.InDelta(t, 1, n.Value.Z, 1e-12)

	decadtest.MeasuresBounds(t, patch, r3.NewVec(0, 0, 0), r3.NewVec(100, 60, 0), decadtest.Exactly())
}

// TestPatchWithHoleNetArea is T2's holed variant: a 100x60 mm rectangle with
// a circular hole, net area with holes subtracted.
func TestPatchWithHoleNetArea(t *testing.T) {
	t.Parallel()
	s, p := rectWithHoleSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(s, p)
	require.NoError(t, err)

	area, err := patch.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Greater(t, area.Bound.Base(), 0.0)
	want := 6000 - 100*math.Pi
	require.LessOrEqual(t, math.Abs(area.Value.Base()-want), area.Bound.Base())

	faces, err := decad.Faces(decad.Planar()).Exactly(1).SelectFaces(patch)
	require.NoError(t, err)
	loops := faces[0].Loops()
	require.Len(t, loops, 2)
	require.True(t, loops[0].IsOuter())
	require.False(t, loops[1].IsOuter())

	free, err := decad.Edges(decad.Free()).Exactly(5).SelectEdges(patch)
	require.NoError(t, err)
	var outerConvex, holeConcave int
	for _, e := range free {
		if _, ok := e.Curve().(decad.Circle3); ok {
			require.False(t, e.IsConvex(), "the hole's rim is concave")
			holeConcave++
			continue
		}
		require.True(t, e.IsConvex(), "the outer rim stays convex")
		outerConvex++
	}
	require.Equal(t, 4, outerConvex)
	require.Equal(t, 1, holeConcave)
}

// TestPatchReproducesThroughPlacement is §5.1's Placed/Duplicate/PlacedCopy
// claim: each re-evaluates the payload and reproduces the patch with no
// further code.
func TestPatchReproducesThroughPlacement(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(s, p)
	require.NoError(t, err)

	motion, err := r3.Translation(r3.NewVec(50, 0, 0))
	require.NoError(t, err)

	placed, err := patch.Placed(motion)
	require.NoError(t, err)
	decadtest.MeasuresArea(t, placed, units.SquareMillimeters(6000), decadtest.Exactly())
	decadtest.MeasuresBounds(t, placed, r3.NewVec(50, 0, 0), r3.NewVec(150, 60, 0), decadtest.Exactly())

	dup, err := placed.Duplicate()
	require.NoError(t, err)
	decadtest.MeasuresArea(t, dup, units.SquareMillimeters(6000), decadtest.Exactly())

	copyMotion, err := r3.Translation(r3.NewVec(0, 50, 0))
	require.NoError(t, err)
	copied, err := dup.PlacedCopy(copyMotion)
	require.NoError(t, err)
	decadtest.MeasuresArea(t, copied, units.SquareMillimeters(6000), decadtest.Exactly())
	decadtest.MeasuresBounds(t, copied, r3.NewVec(50, 50, 0), r3.NewVec(150, 110, 0), decadtest.Exactly())
}

// TestPatchVerifiesUndecided is docs/surface-design.md §9.1's holding fix,
// modelled on TestSurfaceExtrudeSheetVerifiesUndecided (surface_test.go): the
// manifold-with-boundary audit lands later, so a patch reads ValidityUndecided
// rather than proven invalid, and carries no Region and no DiagInvalidBody.
func TestPatchVerifiesUndecided(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(s, p)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br := decadtest.FindBodyReport(t, report, patch)
	require.Equal(t, decad.ValidityUndecided, br.Validity.Outcome)
	require.Nil(t, br.Region)
	require.Len(t, br.Validity.Diagnostics, 1)
	require.Equal(t, decad.DiagUndecidedValidity, br.Validity.Diagnostics[0].Code)
	for _, d := range br.Diagnostics {
		require.NotEqual(t, decad.DiagInvalidBody, d.Code)
	}
}

// TestPatchRejections is docs/surface-design.md Table R rows R2 and R3: a
// foreign, stale or nil profile, and a profile whose recorded boundary
// carries a free-form segment this evaluator cannot integrate. Every
// rejection leaves the document untouched.
func TestPatchRejections(t *testing.T) {
	t.Parallel()

	t.Run("foreign profile", func(t *testing.T) {
		t.Parallel()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 0, 40, 30)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		prof := s.Profiles()[0]

		other, err := w.CreateSketch(w.XZ())
		require.NoError(t, err)
		doc := decad.New()
		_, err = doc.Patch(other, prof)
		require.ErrorIs(t, err, decad.ErrForeignProfile)
		require.Empty(t, doc.Bodies())
	})

	t.Run("stale profile", func(t *testing.T) {
		t.Parallel()
		s, p := plateSketch(t)
		doc := decad.New()
		s.AddConstraint(sketch.NewDistance(s.Points()[0], s.Points()[1], 55))
		_, err := s.Solve(t.Context())
		require.NoError(t, err)
		_, err = doc.Patch(s, p)
		require.ErrorIs(t, err, decad.ErrStaleProfile)
		require.Empty(t, doc.Bodies())
	})

	t.Run("nil profile", func(t *testing.T) {
		t.Parallel()
		s, _ := plateSketch(t)
		doc := decad.New()
		_, err := doc.Patch(s, nil)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Empty(t, doc.Bodies())
	})

	t.Run("unintegrable free-form segment", func(t *testing.T) {
		t.Parallel()
		// Tier C: an unequal-weight NURBS section this evaluator cannot
		// integrate, reusing extrude_freeform_test.go's own fixture shape.
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		p0 := s.CreatePoint(0, 0)
		p1 := s.CreatePoint(1, 2)
		p2 := s.CreatePoint(2, 0)
		_, err = s.CreateNURBS(2, []*sketch.Point{p0, p1, p2}, []float64{1, 2, 1}, []float64{0, 0, 0, 1, 1, 1})
		require.NoError(t, err)
		s.CreateLine(p2, p0)
		profiles := s.Profiles()
		require.Len(t, profiles, 1)

		doc := decad.New()
		_, err = doc.Patch(s, profiles[0])
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Empty(t, doc.Bodies())
	})
}

// TestPatchContextCancellationLeavesDocumentUnchanged mirrors
// TestSweepContextCancellationLeavesDocumentUnchanged (sweep_test.go).
func TestPatchContextCancellationLeavesDocumentUnchanged(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()

	_, err := doc.PatchContext(ctx, s, p)
	require.ErrorIs(t, err, context.Canceled)
	require.Empty(t, doc.Bodies())

	var nilContext context.Context
	_, err = doc.PatchContext(nilContext, s, p)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())
}
