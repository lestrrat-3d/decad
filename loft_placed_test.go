package decad_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/loft-design.md §12 PR 2a's public-surface placement
// tests: Placed/Duplicate/PlacedCopy over the landed loft payload's own
// proven displacement term delta (§5, §8). loft_build_internal_test.go and
// loft_moments_internal_test.go cover the evaluator/accumulator internals;
// this file needs a live sketch, the public entry points, or the recorded
// step.

// requireBitIdentical asserts a and b agree bit for bit on every
// coordinate, never merely to within a tolerance — -0.0 and 0.0 compare
// equal under ==, which the identity fast path's premise cannot afford to
// miss.
func requireBitIdentical(t *testing.T, want, got r3.Vec) {
	t.Helper()
	require.Equal(t, math.Float64bits(want.X), math.Float64bits(got.X), "X")
	require.Equal(t, math.Float64bits(want.Y), math.Float64bits(got.Y), "Y")
	require.Equal(t, math.Float64bits(want.Z), math.Float64bits(got.Z), "Z")
}

// TestR3IdentityApplyIsBitIdentical pins the identity fast path's own
// premise (docs/loft-design.md §5, §12 PR 2a): loftPayload.delta is 0 only
// because r3.Identity().Apply is bit-exact on every finite coordinate,
// including the corners a naive implementation might round differently — a
// subnormal, a near-overflow magnitude, negative zero, and a non-terminating
// binary fraction.
func TestR3IdentityApplyIsBitIdentical(t *testing.T) {
	id := r3.Identity()
	for _, tc := range []struct {
		name string
		v    r3.Vec
	}{
		{"subnormal", r3.NewVec(math.SmallestNonzeroFloat64, math.SmallestNonzeroFloat64, math.SmallestNonzeroFloat64)},
		{"near max", r3.NewVec(1e308, -1e308, 1e308)},
		{"one third", r3.NewVec(1.0/3.0, 1.0/3.0, 1.0/3.0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			requireBitIdentical(t, tc.v, id.Apply(tc.v))
		})
	}

	// Negative zero is checked by VALUE and by DISPLACEMENT, never by sign
	// bit: ApplyDir's linear combination sums cross terms that are
	// individually +0/-0 (e.g. ey.Scale(y).X for x=-0), and -0.0+0.0 rounds
	// to +0.0 under IEEE 754 — so identity's OWN zero-cross terms wash the
	// sign of zero out even though the coordinate is otherwise untouched.
	// That is a fact about float addition, not a rounding the fast path's
	// premise depends on: delta == 0 asserts that the held vertex sits at
	// zero distance from the exact placed image, and -0.0 and 0.0 are the
	// same coordinate at zero distance from each other. Every equality this
	// package runs on a float (exactnessOf, the xform == r3.Identity() fast
	// path itself) reads the same way. docs/loft-design.md §13 states this
	// case as coordinate equality for exactly that reason.
	t.Run("negative zero", func(t *testing.T) {
		v := r3.NewVec(math.Copysign(0, -1), 0, math.Copysign(0, -1))
		got := id.Apply(v)
		require.Equal(t, v, got)
		require.Zero(t, got.Sub(v).Len(), "the identity moves a -0.0 coordinate by exactly zero")
	})
}

// TestLoftPlacementDoesNotRerunTheSeamGates pins docs/loft-design.md §4's
// "S9, S10, S11 and S4's ARITY half belong to the original call alone": a
// placement rebuilds from the payload's already-authenticated records and
// never touches a live sketch, so a profile that goes stale AFTER the body
// is built refuses a fresh Document.Loft (S9) while Duplicate/PlacedCopy/
// Placed of that body still succeed and reproduce its volume.
func TestLoftPlacementDoesNotRerunTheSeamGates(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20) // two 40x40 squares, h=10 -> 16000 mm3 box
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	// Both source profiles go stale only after the loft is recorded.
	s0.AddConstraint(sketch.NewDistance(s0.Points()[0], s0.Points()[1], 55))
	_, err = s0.Solve(t.Context())
	require.NoError(t, err)
	s1.AddConstraint(sketch.NewDistance(s1.Points()[0], s1.Points()[1], 55))
	_, err = s1.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, p0.IsStale())
	require.True(t, p1.IsStale())

	// S9 still gates the entry point: a fresh Loft on the same profiles refuses.
	fresh, err := doc.Loft(s0, p0, s1, p1)
	require.Nil(t, fresh)
	require.ErrorIs(t, err, decad.ErrStaleProfile)

	move, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)

	dup, err := body.Duplicate()
	require.NoError(t, err)
	copied, err := body.PlacedCopy(move)
	require.NoError(t, err)
	// Placed retires the receiver, so it runs last.
	placed, err := body.Placed(move)
	require.NoError(t, err)

	for _, tc := range []struct {
		name string
		b    *decad.Body
	}{{"duplicate", dup}, {"placed copy", copied}, {"placed", placed}} {
		t.Run(tc.name, func(t *testing.T) {
			vol, err := tc.b.Volume()
			require.NoError(t, err)
			require.True(t, vol.Value.Equal(units.CubicMillimeters(16000), 1e-9),
				"a placement rebuilds from the recorded section, not the stale profile; got %s", vol.Value)
		})
	}
}

// TestLoftDuplicateIsMeasurementIdentical proves Duplicate's identity
// motion carries delta 0 (the identity fast path): every measurement and
// every held vertex is bit-identical to the source's, the source stays
// live, and the document gains a second body.
func TestLoftDuplicateIsMeasurementIdentical(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20) // two 40x40 squares, h=10 -> 16000 mm3 box
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	dup, err := body.Duplicate()
	require.NoError(t, err)

	srcVol, err := body.Volume()
	require.NoError(t, err)
	dupVol, err := dup.Volume()
	require.NoError(t, err)
	require.Equal(t, srcVol, dupVol)
	require.Equal(t, decad.Exact, dupVol.Exactness)
	require.InDelta(t, 16000.0, dupVol.Value.Base(), 1e-9)

	srcCentroid, err := body.Centroid()
	require.NoError(t, err)
	dupCentroid, err := dup.Centroid()
	require.NoError(t, err)
	require.Equal(t, srcCentroid, dupCentroid)
	require.Equal(t, decad.Exact, dupCentroid.Exactness)

	srcBounds, err := body.Bounds()
	require.NoError(t, err)
	dupBounds, err := dup.Bounds()
	require.NoError(t, err)
	require.Equal(t, srcBounds, dupBounds)
	require.Equal(t, decad.Exact, dupBounds.Exactness)

	srcVerts, dupVerts := body.Vertices(), dup.Vertices()
	require.Len(t, dupVerts, len(srcVerts))
	for i := range srcVerts {
		requireBitIdentical(t, srcVerts[i].Position().Value, dupVerts[i].Position().Value)
	}

	require.Equal(t, []*decad.Body{body, dup}, doc.Bodies(), "the source stays live")
}

// TestLoftPlacedRotationSoundness is the regression docs/loft-design.md's
// PR 2a exists for: rotating the 16000 mm3 square-square loft 37 degrees
// about X. The naive re-lift-and-round implementation this design measured
// publishes Volume = 16000.000000000002 with Bound = 7.203e-13 — an
// interval that MISSES the true volume by 1.819e-12 mm3, because every held
// vertex is exact ONLY under the identity transform (§5), and Bounds still
// published Exact with a zero bound over vertices a rotation rounded.
func TestLoftPlacedRotationSoundness(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	rot, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(37))
	require.NoError(t, err)
	placed, err := body.Placed(rot)
	require.NoError(t, err)

	vol, err := placed.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(vol.Value.Base()-16000.0), vol.Bound.Base(),
		"the published interval must enclose the true volume — the naive implementation misses by 1.819e-12 against a 7.203e-13 bound")
	require.Equal(t, decad.Approximate, vol.Exactness)

	// The closed-form rotated centroid: the unplaced box's own centroid is
	// (0, 0, 5); rotating by theta about X sends (y, z) to
	// (y*cos(theta) - z*sin(theta), y*sin(theta) + z*cos(theta)).
	theta := 37.0 * math.Pi / 180.0
	wantCentroid := r3.NewVec(0, -5*math.Sin(theta), 5*math.Cos(theta))
	c, err := placed.Centroid()
	require.NoError(t, err)
	require.LessOrEqual(t, c.Value.Sub(wantCentroid).Len(), c.Bound.Base(),
		"the placed centroid must enclose the closed-form rotated centroid")

	for _, v := range placed.Vertices() {
		require.GreaterOrEqual(t, v.Position().Bound.Base(), 0.0)
	}

	bounds, err := placed.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, bounds.Exactness)
}

// TestLoftPlacedCopyTranslation proves a pure translation moves Bounds and
// Centroid by exactly the translation, keeps the volume value, publishes a
// positive bound enclosing the true 16000 mm3, and leaves the source live.
func TestLoftPlacedCopyTranslation(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	srcBounds, err := body.Bounds()
	require.NoError(t, err)
	srcCentroid, err := body.Centroid()
	require.NoError(t, err)
	srcVol, err := body.Volume()
	require.NoError(t, err)

	shift := r3.NewVec(100, 0, 0)
	move, err := r3.Translation(shift)
	require.NoError(t, err)
	copied, err := body.PlacedCopy(move)
	require.NoError(t, err)

	copiedBounds, err := copied.Bounds()
	require.NoError(t, err)
	require.InDelta(t, srcBounds.Min.X+100, copiedBounds.Min.X, 1e-9)
	require.InDelta(t, srcBounds.Max.X+100, copiedBounds.Max.X, 1e-9)
	require.InDelta(t, srcBounds.Min.Y, copiedBounds.Min.Y, 1e-9)
	require.InDelta(t, srcBounds.Min.Z, copiedBounds.Min.Z, 1e-9)

	copiedCentroid, err := copied.Centroid()
	require.NoError(t, err)
	require.InDelta(t, srcCentroid.Value.X+100, copiedCentroid.Value.X, 1e-9)
	require.InDelta(t, srcCentroid.Value.Y, copiedCentroid.Value.Y, 1e-9)
	require.InDelta(t, srcCentroid.Value.Z, copiedCentroid.Value.Z, 1e-9)

	copiedVol, err := copied.Volume()
	require.NoError(t, err)
	require.Equal(t, srcVol.Value, copiedVol.Value, "a translation commits no rounding to the volume's own VALUE")
	require.Positive(t, copiedVol.Bound.Base())
	require.LessOrEqual(t, math.Abs(copiedVol.Value.Base()-16000.0), copiedVol.Bound.Base())

	require.Equal(t, []*decad.Body{body, copied}, doc.Bodies(), "the source stays live")
}

// TestLoftPlacedReflectionHerringbone mirrors the box loft: the result has
// positive volume, its face normals still point outward (spot-checked
// against Face.NormalAt on one wall), face/edge/vertex counts are
// unchanged, and the centroid mirrors the source's within bound. §5's
// whole-shell orientation step re-decides the sign from the reflected
// triangle set on its own, so no separate winding-flip case is needed.
func TestLoftPlacedReflectionHerringbone(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	mirrorFrame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	mirror, err := r3.Reflection(mirrorFrame)
	require.NoError(t, err)
	require.True(t, mirror.IsReflection())

	mirrored, err := body.PlacedCopy(mirror)
	require.NoError(t, err)

	vol, err := mirrored.Volume()
	require.NoError(t, err)
	require.Positive(t, vol.Value.Base())
	require.LessOrEqual(t, math.Abs(vol.Value.Base()-16000.0), vol.Bound.Base())

	require.Equal(t, len(body.Faces()), len(mirrored.Faces()))
	require.Equal(t, len(body.Edges()), len(mirrored.Edges()))
	require.Equal(t, len(body.Vertices()), len(mirrored.Vertices()))

	// Spot-check outward normals on one wall: every wall vertex's world
	// position sits at x <= 0 (the source occupies x in [-20, 20], mirrored
	// across x=0 sends it to [-20, 20] reflected — x -> -x — so a point
	// originally at the max-x corner now sits at min-x). The material is
	// still on the box's interior side of every wall, so the outward normal
	// at that corner points away from the box's own centroid.
	centroid, err := mirrored.Centroid()
	require.NoError(t, err)
	for _, f := range mirrored.Faces() {
		if _, ok := f.Surface().(decad.Plane); !ok {
			continue
		}
		loops := f.Loops()
		if len(loops) == 0 || len(loops[0].CoEdges()) == 0 {
			continue
		}
		p := loops[0].CoEdges()[0].Start().Position().Value
		n, err := f.NormalAt(p)
		require.NoError(t, err)
		outward := p.Sub(centroid.Value)
		if outward.Len() < 1e-9 {
			continue
		}
		require.Greater(t, n.Value.Dot(outward), 0.0, "the outward normal must point away from the body's own interior")
		break
	}

	srcCentroid, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, -srcCentroid.Value.X, centroid.Value.X, 1e-9)
	require.InDelta(t, srcCentroid.Value.Y, centroid.Value.Y, 1e-9)
	require.InDelta(t, srcCentroid.Value.Z, centroid.Value.Z, 1e-9)

	require.Equal(t, []*decad.Body{body, mirrored}, doc.Bodies())
}

// TestLoftPlacedCopyChainDoesNotAccumulate proves the re-lift-from-record
// path (docs/loft-design.md §7, §12 PR 2a): ten successive PlacedCopy pure
// rotations, chained one onto the last, keep the volume bound within a
// small constant factor of the FIRST placement's own, since each
// evaluation re-lifts from the two ORIGINAL recorded profiles rather than
// moving an already-placed mesh — never accumulating rounding across the
// chain the way facetedPayload.placed's move-the-held-mesh path would.
func TestLoftPlacedCopyChainDoesNotAccumulate(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	rot, err := r3.Rotation(r3.NewVec(0, 1, 0), units.Degrees(11))
	require.NoError(t, err)

	cur := body
	var firstBound float64
	for i := range 10 {
		cur, err = cur.PlacedCopy(rot)
		require.NoError(t, err)
		vol, err := cur.Volume()
		require.NoError(t, err)
		require.Positive(t, vol.Bound.Base())
		if i == 0 {
			firstBound = vol.Bound.Base()
			continue
		}
		require.LessOrEqual(t, vol.Bound.Base(), firstBound*8,
			"delta must not grow across a chain of placements — each re-lifts from the original record")
	}
}

// TestLoftPlacedTopologyAndRoles proves a placed body's faces carry the same
// role grammar under the NEW step, resolves exactly one capStart face
// through the selector vocabulary, stays manifold, and matches the source's
// per-edge convexity (§5's junction rule reads the SAME geometric relation
// under any rigid motion).
func TestLoftPlacedTopologyAndRoles(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	move, err := r3.Translation(r3.NewVec(50, -25, 5))
	require.NoError(t, err)
	placed, err := body.Placed(move)
	require.NoError(t, err)

	newStep := placed.Origin().Step
	for _, f := range placed.Faces() {
		for _, o := range f.Origins() {
			require.Equal(t, newStep, o.Step)
		}
	}

	faces, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(placed))).SelectFaces(placed)
	require.NoError(t, err)
	require.Len(t, faces, 1)

	for _, e := range placed.Edges() {
		require.Len(t, e.Faces(), 2, "every edge of a loft bounds exactly two faces")
	}

	srcEdges := body.Edges()
	placedEdges := placed.Edges()
	require.Len(t, placedEdges, len(srcEdges))
	for i := range srcEdges {
		require.Equal(t, srcEdges[i].IsConvex(), placedEdges[i].IsConvex(),
			"a placement's own junction turn reads the same geometric relation under any rigid motion")
	}

	require.Equal(t, []*decad.Body{placed}, doc.Bodies(), "Placed retires the receiver")
}

// TestLoftPlacedFaceAreaSumMatchesBodyArea catches a per-face bound that
// forgot the perturbation term: the sum of every face's own Area() must
// equal Body.Area().Value within the summed bounds.
func TestLoftPlacedFaceAreaSumMatchesBodyArea(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	rot, err := r3.Rotation(r3.NewVec(1, 1, 1), units.Degrees(23))
	require.NoError(t, err)
	placed, err := body.Placed(rot)
	require.NoError(t, err)

	bodyArea, err := placed.Area()
	require.NoError(t, err)

	sum, sumBound := 0.0, 0.0
	for _, f := range placed.Faces() {
		a, err := f.Area()
		require.NoError(t, err)
		sum += a.Value.Base()
		sumBound += a.Bound.Base()
	}
	require.InDelta(t, bodyArea.Value.Base(), sum, bodyArea.Bound.Base()+sumBound+1e-9)
}

// TestLoftPlacedAccessorExactness pins docs/loft-design.md §8's per-accessor
// rule on both sides of delta. An unplaced loft's every vertex position is
// Exact with a zero bound; a placed one's is Approximate carrying the
// payload's own delta — the SAME bound at every vertex — and its every edge
// length and face area carry a positive delta term on top of their own
// square-root bound, so neither can be Exact however exactly its own
// evaluation comes out. The receiver here is a 40x40x10 box whose edges are
// 40 and 10, both exactly representable, so an unplaced fixture cannot tell a
// missing delta term from an exact square root; only the placed body can.
func TestLoftPlacedAccessorExactness(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	for _, v := range body.Vertices() {
		pos := v.Position()
		require.Equal(t, decad.Exact, pos.Exactness, "an unplaced loft vertex is Exact by construction")
		require.Zero(t, pos.Bound.Base())
	}

	move, err := r3.Translation(r3.NewVec(50, -25, 5))
	require.NoError(t, err)
	placed, err := body.Placed(move)
	require.NoError(t, err)

	verts := placed.Vertices()
	require.NotEmpty(t, verts)
	delta := verts[0].Position().Bound.Base()
	require.Positive(t, delta, "a non-identity placement carries a positive delta")
	for _, v := range verts {
		pos := v.Position()
		require.Equal(t, decad.Approximate, pos.Exactness)
		require.Equal(t, delta, pos.Bound.Base(), "every placed vertex publishes the payload's own delta")
	}

	for _, e := range placed.Edges() {
		length, err := e.Length()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, length.Exactness)
		require.Greater(t, length.Bound.Base(), delta,
			"a placed edge adds its own delta term on top of the square root's bound")
	}

	for _, f := range placed.Faces() {
		area, err := f.Area()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, area.Exactness)
		require.Positive(t, area.Bound.Base())
	}
}

// TestLoftPlacedRetireAndLiveness proves Placed retires the receiver while
// Duplicate/PlacedCopy leave it live, and a refused placement — an invalid
// (zero-value) transform, and an S12 fixture whose proven volume allowance
// swamps its tiny held volume — leaves the recipe and document untouched.
func TestLoftPlacedRetireAndLiveness(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	move, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	placed, err := body.Placed(move)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{placed}, doc.Bodies(), "Placed retires the receiver")

	dup, err := placed.Duplicate()
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{placed, dup}, doc.Bodies(), "Duplicate leaves the receiver live")

	copied, err := dup.PlacedCopy(move)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{placed, dup, copied}, doc.Bodies(), "PlacedCopy leaves the receiver live")

	bodiesBefore := doc.Bodies()
	stepsBefore := doc.Recipe().Steps

	invalid, err := copied.Placed(r3.Transform{})
	require.Nil(t, invalid)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, bodiesBefore, doc.Bodies())
	require.Equal(t, stepsBefore, doc.Recipe().Steps)
}

// TestLoftPlacedS12TinyBodyFarTranslation refuses a placement whose proven
// volume allowance is not smaller than the held volume (Table S, S12): a
// vanishingly small loft placed by an enormous translation carries a delta
// whose swept-volume term swamps its own tiny volume, leaving the
// centroid's quotient allowance with no positive denominator to divide by.
func TestLoftPlacedS12TinyBodyFarTranslation(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 1e-4, 1e-4)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Positive(t, vol.Value.Base())

	stepsBefore := doc.Recipe().Steps
	bodiesBefore := doc.Bodies()

	far, err := r3.Translation(r3.NewVec(1e10, 0, 0))
	require.NoError(t, err)
	placed, err := body.Placed(far)
	require.Nil(t, placed)
	require.ErrorIs(t, err, decad.ErrUnsupported)

	require.Equal(t, bodiesBefore, doc.Bodies(), "a refused S12 placement leaves the document untouched")
	require.Equal(t, stepsBefore, doc.Recipe().Steps, "a refused S12 placement leaves the recipe untouched")
}

// TestLoftPlacedS13OverflowingCoordinate refuses a placement whose composed
// motion carries a lifted section coordinate past the representable float64
// range (Table S, S13). The fixture lofts between the XY plane and a plane
// offset 1e300 from it — a body decad builds without complaint — then
// translates it by MaxFloat64 along Z, which overflows the top section's own
// coordinates.
//
// The refusal must be a RETURNED error and never a recovered panic: the loft's
// orientation sum and mass accumulator lift every coordinate into an exact
// rational, and that lift is defined only on a finite float, so the gate has
// to run while the coordinate is still a float. ErrUnsupported, not
// ErrNotFinite: every input here is finite, and only decad's own float
// evaluation of the lift runs off the range.
func TestLoftPlacedS13OverflowingCoordinate(t *testing.T) {
	s0, p0, s1, p1 := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 5, 1e300)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err, "the unplaced far-plane loft builds")

	stepsBefore := doc.Recipe().Steps
	bodiesBefore := doc.Bodies()

	far, err := r3.Translation(r3.NewVec(0, 0, math.MaxFloat64))
	require.NoError(t, err)
	placed, err := body.Placed(far)
	require.Nil(t, placed)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.NotErrorIs(t, err, decad.ErrNotFinite)
	require.Contains(t, err.Error(), "representable float64 range")

	require.Equal(t, bodiesBefore, doc.Bodies(), "a refused S13 placement leaves the document untouched")
	require.Equal(t, stepsBefore, doc.Recipe().Steps, "a refused S13 placement leaves the recipe untouched")
}

// TestLoftPlacedContextCancellation proves a canceled context returns
// ctx.Err() with the receiver still live and the recipe unchanged.
func TestLoftPlacedContextCancellation(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	stepsBefore := doc.Recipe().Steps

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	move, err := r3.Translation(r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	placed, err := body.PlacedContext(ctx, move)
	require.Nil(t, placed)
	require.ErrorIs(t, err, context.Canceled)

	require.Equal(t, []*decad.Body{body}, doc.Bodies(), "the receiver stays live")
	require.Equal(t, stepsBefore, doc.Recipe().Steps)
}

// TestLoftPlacedVerifySound proves a placed loft is Sound at the default
// tolerance, and two lofts placed apart read box-disjoint Sound.
func TestLoftPlacedVerifySound(t *testing.T) {
	s0, p0, s1, p1 := loftSquares(t, 20, 20)
	doc := decad.New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)

	rot, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(37))
	require.NoError(t, err)
	placed, err := body.Placed(rot)
	require.NoError(t, err)
	require.NotNil(t, placed)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())

	t0, tp0, t1, tp1 := loftSquaresAt(t, r3.NewVec(1000, 0, 0), 20, 20, 10)
	other, err := doc.Loft(t0, tp0, t1, tp1)
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(2000, 0, 0))
	require.NoError(t, err)
	otherPlaced, err := other.Placed(move)
	require.NoError(t, err)

	report, err = doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Interferences)
	require.True(t, report.Trustworthy())
	_ = otherPlaced
}
