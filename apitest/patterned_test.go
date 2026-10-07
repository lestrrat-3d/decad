package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The Patterned tests of docs/mirror-pattern-design.md §8: one body holding
// every instance, a prism group where sketch proves the instances disjoint
// and Union's own result otherwise.

// TestPatternedLinearGroup is §8's exact linear pattern under Patterned: four
// 5 mm pegs at 10 mm steps are one prism group of four lumps and 24 faces,
// Exact 500 mm³, and the receiver is retired.
func TestPatternedLinearGroup(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	peg := sc.box(t, sc.w.XY(), 0, 0, 5, 5, mirrorAlong(5))
	group, err := peg.Patterned(t.Context(), decad.LinearPattern{Dir: patternX, Step: units.Millimeters(10), Count: 4})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{group}, sc.doc.Bodies())
	require.False(t, anyFaceIsFaceted(group))
	require.Len(t, group.Lumps(), 4)
	require.Len(t, group.Faces(), 24)
	v, err := group.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, v.Exactness)
	require.Equal(t, 500.0, volumeMM(t, v))
	starts, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(group))).AtLeast(1).SelectFaces(group)
	require.NoError(t, err)
	require.Len(t, starts, 4, "CapStart selects every lump's bottom face")
	requireMirrorBox(t, group, r3.NewVec(0, 0, 0), r3.NewVec(35, 5, 5))
	requireMeshWatertightAt(t, group, 0.1)
}

// TestPatternedCircularGroup is §8's circular pattern under Patterned: six Ø4
// pins on a 20 mm radius are one group of six lumps enclosing 6·4π·5 mm³, and
// four 4×2 blocks on the exact quarter turns are Exact.
func TestPatternedCircularGroup(t *testing.T) {
	t.Parallel()
	t.Run("six pins", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		pin := sc.cylinder(t, sc.w.XY(), 20, 0, 2, mirrorAlong(5))
		group, err := pin.Patterned(t.Context(), decad.CircularPattern{Axis: patternZ, Count: 6})
		require.NoError(t, err)
		require.False(t, anyFaceIsFaceted(group))
		require.Len(t, group.Lumps(), 6)
		v, err := group.Volume()
		require.NoError(t, err)
		requirePiLinearEnclosed(t, v, 0, 120)
	})
	t.Run("four blocks", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		block := sc.box(t, sc.w.XY(), 18, -1, 22, 1, mirrorAlong(5))
		group, err := block.Patterned(t.Context(), decad.CircularPattern{Axis: patternZ, Count: 4})
		require.NoError(t, err)
		require.Len(t, group.Lumps(), 4)
		v, err := group.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, v.Exactness)
		require.Equal(t, 160.0, volumeMM(t, v))
		c, err := group.Centroid()
		require.NoError(t, err)
		require.Equal(t, r3.NewVec(0, 0, 2.5), c.Value)
	})
}

// TestPatternedNHoleCut is §8's N-hole cut: a Ø4 disc patterned six times at
// 10 mm is one group tool, and a 60×40×5 plate cut by it is analytic, one
// lump with twelve faces, its volume within its bound of 12000 − 6·π·4·5,
// filletable, and measured by the wall survey.
func TestPatternedNHoleCut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, -20, 60, 20, 5)
	disc := discBody(t, doc, 5, 2, 5)
	tool, err := disc.Patterned(t.Context(), decad.LinearPattern{Dir: patternX, Step: units.Millimeters(10), Count: 6})
	require.NoError(t, err)
	require.Len(t, tool.Lumps(), 6)

	got, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Lumps(), 1)
	require.Len(t, got.Faces(), 12)
	volume, err := got.Volume()
	require.NoError(t, err)
	requirePiLinearEnclosed(t, volume, 12000, -120)
	require.Less(t, boundMM3(t, volume), 1e-9)
	requireMeshWatertightAt(t, got, 0.05)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.NotNil(t, report.Bodies[0].Wall.Minimum, "the wall survey measures the holed plate")
	filleted, err := got.Fillet(t.Context(), verticalConvexEdge(), units.Millimeters(1))
	require.NoError(t, err)
	fv, err := filleted.Volume()
	require.NoError(t, err)
	require.Less(t, volumeMM(t, fv), volumeMM(t, volume))
}

// unionOutcome is what a Union of a and b returns, reduced to what two
// independent runs can compare: the error text, or the volume, its
// exactness, the lump count and the face count.
type unionOutcome struct {
	err       string
	volume    float64
	exactness decad.Exactness
	lumps     int
	faces     int
}

func outcomeOf(t *testing.T, b *decad.Body, err error) unionOutcome {
	t.Helper()
	if err != nil {
		return unionOutcome{err: err.Error()}
	}
	v, verr := b.Volume()
	require.NoError(t, verr)
	return unionOutcome{volume: volumeMM(t, v), exactness: v.Exactness, lumps: len(b.Lumps()), faces: len(b.Faces())}
}

// TestPatternedFallsBackToUnion is §6.1's rule 3: instances sketch does not
// prove disjoint combine by Union, and Patterned returns exactly what Union
// returns for the receiver and the instance PatternCopies builds from the same
// spec on a twin document. Two discs crossing about the origin join; pegs that
// overlap or share a wall reach the collinear-carrier refusal until
// general-boolean class A3 lands; pegs stacked end to end along their own
// sweep (the non-co-directional row) instance through PlacedCopy, whose
// placement keeps them off the analytic stacked union, and reach the mesh
// path's coplanar refusal. A refusal leaves the
// document unchanged and the receiver live.
func TestPatternedFallsBackToUnion(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		Name  string
		Build func(*testing.T, *mirrorScene) *decad.Body
		Spec  decad.PatternSpec
	}{
		{Name: "crossing discs", Spec: decad.CircularPattern{Axis: patternZ, Count: 2},
			Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
				return sc.cylinder(t, sc.w.XY(), 2, 0, 3, mirrorAlong(5))
			}},
		{Name: "overlapping pegs", Spec: decad.LinearPattern{Dir: patternX, Step: units.Millimeters(10), Count: 2},
			Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
				return sc.box(t, sc.w.XY(), 0, 0, 15, 5, mirrorAlong(5))
			}},
		{Name: "pegs sharing a wall", Spec: decad.LinearPattern{Dir: patternX, Step: units.Millimeters(10), Count: 2},
			Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
				return sc.box(t, sc.w.XY(), 0, 0, 10, 5, mirrorAlong(5))
			}},
		{Name: "pegs along the sweep", Spec: decad.LinearPattern{Dir: patternZ, Step: units.Millimeters(5), Count: 2},
			Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
				return sc.box(t, sc.w.XY(), 0, 0, 5, 5, mirrorAlong(5))
			}},
	} {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			twin := newMirrorScene()
			a := tc.Build(t, twin)
			copies, err := a.PatternCopies(t.Context(), tc.Spec)
			require.NoError(t, err)
			require.Len(t, copies, 1)
			union, err := decad.Union(t.Context(), a, copies[0])
			want := outcomeOf(t, union, err)

			sc := newMirrorScene()
			receiver := tc.Build(t, sc)
			got, err := receiver.Patterned(t.Context(), tc.Spec)
			require.Equal(t, want, outcomeOf(t, got, err))
			if err != nil {
				require.Equal(t, []*decad.Body{receiver}, sc.doc.Bodies(), "a refused Union leaves the receiver live")
				return
			}
			require.Equal(t, []*decad.Body{got}, sc.doc.Bodies())
			require.Len(t, got.Lumps(), 1)
		})
	}
}

// TestPatternedRefusesDisjointStacks is §6.1's rule 2: a blind-cut plate
// patterned into disjoint instances has no payload holding them, so Patterned
// is ErrUnsupported and the document is unchanged; PatternCopies serves.
func TestPatternedRefusesDisjointStacks(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	plate := sc.box(t, sc.w.XY(), 0, 0, 10, 10, mirrorAlong(10))
	top, err := sc.w.CreateOffsetPlane(sc.w.XY(), 6)
	require.NoError(t, err)
	pocket := sc.box(t, top, 3, 3, 7, 7, mirrorAlong(4))
	part, err := decad.Cut(t.Context(), plate, pocket)
	require.NoError(t, err)
	_, err = part.Patterned(t.Context(), decad.LinearPattern{Dir: patternX, Step: units.Millimeters(20), Count: 3})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "PatternCopies")
	require.Equal(t, []*decad.Body{part}, sc.doc.Bodies())
}

// TestPatternedGates covers the gates Patterned shares with PatternCopies and
// §8's cancellation row: a context canceled inside the disjointness scene
// returns context.Canceled with the receiver live and nothing registered.
func TestPatternedGates(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	peg := sc.box(t, sc.w.XY(), 0, 0, 5, 5, mirrorAlong(5))
	spec := decad.LinearPattern{Dir: patternX, Step: units.Millimeters(10), Count: 3}

	_, err := peg.Patterned(t.Context(), decad.LinearPattern{Dir: patternX, Step: units.Millimeters(10), Count: 1})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = peg.Patterned(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = peg.Patterned(t.Context(), decad.LinearPattern{Dir: r3.NewVec(math.NaN(), 0, 0), Step: units.Millimeters(10), Count: 2})
	require.ErrorIs(t, err, decad.ErrNotFinite)

	ctx := &operationCancelContext{Context: t.Context(), target: "prismProfilesContext"}
	_, err = peg.Patterned(ctx, spec)
	require.True(t, ctx.entered, "the premise: the disjointness scene ran")
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, []*decad.Body{peg}, sc.doc.Bodies())

	group, err := peg.Patterned(t.Context(), spec)
	require.NoError(t, err)
	_, err = peg.Patterned(t.Context(), spec)
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	require.Equal(t, []*decad.Body{group}, sc.doc.Bodies())
}
