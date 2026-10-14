package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// mirrorJoinedLoft returns the loft mirror join of two squares and its lower
// half, both live in doc.
func mirrorJoinedLoft(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	base := w.XY()
	top, err := w.CreateOffsetPlane(base, 10)
	require.NoError(t, err)
	square := func(plane *sketch.Plane, half float64) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		r := s.CreateRectangle(-half, -half, half, half)
		s.Fix(r.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	s0, p0 := square(base, 2)
	s1, p1 := square(top, 1)
	doc := New()
	lower, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 10), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	upper, err := lower.MirroredCopy(t.Context(), MirrorFrame{Frame: frame})
	require.NoError(t, err)
	joined, err := Union(t.Context(), upper, lower)
	require.NoError(t, err)
	_, ok := joined.payload.(facetedPayload)
	require.True(t, ok)
	return joined
}

func TestCopyRotationRecordsExactPlacementOnDirectCopy(t *testing.T) {
	joined := mirrorJoinedLoft(t)
	zAxis := r3.Vec{Z: 1}
	angle := units.Degrees(30)

	exact, err := Copy(t.Context(), joined, WithCopyRotation(r3.Vec{}, zAxis, angle))
	require.NoError(t, err)
	payload := exact.payload.(facetedPayload)
	require.NotNil(t, payload.copyLineage)
	require.True(t, payload.copyLineage.direct)
	require.NotNil(t, payload.copyLineage.exact)
	require.Nil(t, payload.loftMirrorCopy, "the float-motion proof is dropped")
	require.NotNil(t, payload.mirrorCapGroups, "the cap-role provenance is kept")
	for _, g := range payload.groups {
		require.Nil(t, g.surface)
		require.Nil(t, g.denoted)
		require.Zero(t, g.normalBound)
	}

	// Control: the same motion through WithCopyPlacement keeps the tags and
	// the proof, and records no exact rotation.
	turn, err := r3.RotationAround(r3.Vec{}, zAxis, angle)
	require.NoError(t, err)
	control, err := Copy(t.Context(), joined, WithCopyPlacement(turn))
	require.NoError(t, err)
	cp := control.payload.(facetedPayload)
	require.Nil(t, cp.copyLineage.exact)
	require.NotNil(t, cp.loftMirrorCopy)
	tagged := 0
	for _, g := range cp.groups {
		if g.surface != nil {
			tagged++
		}
	}
	require.Positive(t, tagged)

	// The held mesh is the float placement D applied as ever.
	require.Equal(t, cp.verts, payload.verts)
	require.Equal(t, cp.tris, payload.tris)
}

func TestCopyRotationLineageExactIsDirectOnly(t *testing.T) {
	joined := mirrorJoinedLoft(t)
	turn := WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30))
	first, err := Copy(t.Context(), joined, turn)
	require.NoError(t, err)
	require.NotNil(t, first.payload.(facetedPayload).copyLineage.exact)

	second, err := Copy(t.Context(), first, turn)
	require.NoError(t, err)
	sp := second.payload.(facetedPayload)
	require.NotNil(t, sp.copyLineage)
	require.False(t, sp.copyLineage.direct)
	require.Nil(t, sp.copyLineage.exact, "a copy of a copy records no exact rotation")
	for _, g := range sp.groups {
		require.Nil(t, g.surface, "the second exact placement still drops every tag")
	}

	shift, err := r3.Translation(r3.NewVec(5, 0, 0))
	require.NoError(t, err)
	moved, err := Transform(t.Context(), first, shift)
	require.NoError(t, err)
	mp := moved.payload.(facetedPayload)
	if mp.copyLineage != nil {
		require.Nil(t, mp.copyLineage.exact, "Transform re-binds without the exact rotation")
	}
}

func TestCopyRotationOrdinaryCopyOfExactCopyKeepsNoTags(t *testing.T) {
	joined := mirrorJoinedLoft(t)
	first, err := Copy(t.Context(), joined, WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30)))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(5, 0, 0))
	require.NoError(t, err)
	second, err := Copy(t.Context(), first, WithCopyPlacement(shift))
	require.NoError(t, err)
	sp := second.payload.(facetedPayload)
	require.Nil(t, sp.loftMirrorCopy)
	require.NotNil(t, sp.mirrorCapGroups)
	for _, g := range sp.groups {
		require.Nil(t, g.surface)
	}
}

func TestExactGroupPlacedMapsCentroidAndDropsTags(t *testing.T) {
	q, d, err := exactrotation.New(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(90))
	require.NoError(t, err)
	_ = d
	origin := FeatureRef{producer: 1, Role: roleCapStart}
	g := facetGroup{
		origins: []FeatureRef{origin}, hasCentroid: true, planar: true, reversed: true,
		centroid:    VecMeasurement{Value: r3.NewVec(2, 0, 5), Exactness: Exact},
		surface:     Plane{},
		normalBound: 3,
	}
	require.NoError(t, exactGroupPlaced(&g, q))
	require.Nil(t, g.surface)
	require.Nil(t, g.denoted)
	require.Zero(t, g.normalBound)
	require.True(t, g.planar)
	require.True(t, g.reversed)
	require.Equal(t, []FeatureRef{origin}, g.origins)
	require.InDelta(t, 0, g.centroid.Value.X, 1e-12)
	require.InDelta(t, 2, g.centroid.Value.Y, 1e-12)
	require.InDelta(t, 5, g.centroid.Value.Z, 1e-12)
	// The reading is exactly Q*c: Q is orthogonal, so the stretch charge is 0.
	lin := q.Linear()
	exactY := new(big.Rat).Mul(lin[1][0], big.NewRat(2, 1))
	got := new(big.Rat)
	got.SetFloat64(g.centroid.Value.Y)
	delta, _ := new(big.Rat).Sub(got, exactY).Abs(new(big.Rat)).Float64()
	require.LessOrEqual(t, delta, g.centroid.Bound.Base())
}

func TestCopyRotationExactSourceVertsDropForRotation(t *testing.T) {
	_, _, placed := boundedFacetedFloorFixture(t)
	source := placed.payload.(facetedPayload)
	require.NotEmpty(t, source.exactSourceVerts)

	shift, err := r3.Translation(r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	control, err := Copy(t.Context(), placed, WithCopyPlacement(shift))
	require.NoError(t, err)
	require.NotEmpty(t, control.payload.(facetedPayload).exactSourceVerts)

	rotated, err := Copy(t.Context(), placed, WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30)))
	require.NoError(t, err)
	require.Empty(t, rotated.payload.(facetedPayload).exactSourceVerts)

	identity, err := Copy(t.Context(), placed, WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, units.Radians(0)))
	require.NoError(t, err)
	require.NotEmpty(t, identity.payload.(facetedPayload).exactSourceVerts,
		"an exactly identity record keeps the zero-bound source")
}

// TestPlacedWithChargesTheGapToExactRotation places the held mesh by a float
// transform that is deliberately 1e-6 rad away from the exact record. The
// recorded vertex bounds must still cover every held vertex's exact distance
// to Q*v, which only the gap charge makes true.
func TestPlacedWithChargesTheGapToExactRotation(t *testing.T) {
	joined := mirrorJoinedLoft(t)
	source := joined.payload.(facetedPayload)
	angle := units.Radians(0.5)
	q, _, err := exactrotation.New(r3.Vec{}, r3.Vec{Z: 1}, angle)
	require.NoError(t, err)
	off, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Radians(0.5+1e-6))
	require.NoError(t, err)
	composed, err := source.xform.Then(off)
	require.NoError(t, err)
	body, err := source.placedWith(t.Context(), joined.doc, joined.doc.nextProducerID(), composed, q)
	require.NoError(t, err)
	placed := body.payload.(facetedPayload)
	require.Len(t, placed.verts, len(source.verts))

	lin, tr := q.Linear(), q.Translation()
	ratOf := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	farthest := 0.0
	for i, v := range source.verts {
		in := [3]*big.Rat{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)}
		held := [3]*big.Rat{ratOf(placed.verts[i].X), ratOf(placed.verts[i].Y), ratOf(placed.verts[i].Z)}
		dist2 := new(big.Rat)
		for r := range in {
			image := new(big.Rat).Set(tr[r])
			for c := range in {
				image.Add(image, new(big.Rat).Mul(lin[r][c], in[c]))
			}
			d := image.Sub(image, held[r])
			dist2.Add(dist2, d.Mul(d, d))
		}
		bound := ratOf(placed.vertexBound[i])
		require.LessOrEqual(t, dist2.Cmp(bound.Mul(bound, bound)), 0,
			"vertex %d lies farther from Q*v than its recorded bound", i)
		d, _ := dist2.Float64()
		farthest = max(farthest, d)
	}
	require.Greater(t, farthest, 1e-12, "the offset transform must move vertices measurably off Q")
}

func TestPlacedWithDropsFloatPlacementRecords(t *testing.T) {
	joined := mirrorJoinedLoft(t)
	source := joined.payload.(facetedPayload)
	source.pointSection = &pointSectionRecord{}
	source.pointPlane = &pointPlaneSliceRecord{}
	source.pointSimilarity = &pointPlaneSimilarityRecord{}
	source.pointCone = &pointConeTrimRecord{}
	source.pointPlaneCapDisplacement = 1
	q, d, err := exactrotation.New(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)
	composed, err := source.xform.Then(d)
	require.NoError(t, err)

	body, err := source.placedWith(t.Context(), joined.doc, joined.doc.nextProducerID(), composed, q)
	require.NoError(t, err)
	exact := body.payload.(facetedPayload)
	require.Nil(t, exact.pointSection)
	require.Nil(t, exact.pointPlane)
	require.Nil(t, exact.pointSimilarity)
	require.Nil(t, exact.pointCone)
	require.Nil(t, exact.revolveLoftSector)
	require.Nil(t, exact.loftMirrorCopy)
	require.Zero(t, exact.pointPlaneCapDisplacement)

	// Control: the float placement keeps the section source and records the
	// similarity step and the sector it placed.
	body, err = source.placed(t.Context(), joined.doc, joined.doc.nextProducerID(), composed)
	require.NoError(t, err)
	float := body.payload.(facetedPayload)
	require.NotNil(t, float.pointSection)
	require.NotNil(t, float.pointSimilarity)
	require.NotNil(t, float.revolveLoftSector)
	require.NotNil(t, float.loftMirrorCopy)
}
