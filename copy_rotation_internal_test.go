package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
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
	upper, err := MirroredCopy(t.Context(), lower, MirrorFrame{Frame: frame})
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
	origin := provenance.Ref{Producer: 1, Role: provenance.RoleCapStart}
	g := facetGroup{
		origins: []provenance.Ref{origin}, hasCentroid: true, planar: true, reversed: true,
		centroid:    VecMeasurement{Value: r3.NewVec(2, 0, 5), Exactness: Exact},
		surface:     surfacegeom.Plane{},
		normalBound: 3,
	}
	require.NoError(t, exactGroupPlaced(&g, q))
	require.Nil(t, g.surface)
	require.Nil(t, g.denoted)
	require.Zero(t, g.normalBound)
	require.True(t, g.planar)
	require.True(t, g.reversed)
	require.Equal(t, []provenance.Ref{origin}, g.origins)
	require.InDelta(t, 0, g.centroid.Value.X, 1e-12)
	require.InDelta(t, 2, g.centroid.Value.Y, 1e-12)
	require.InDelta(t, 5, g.centroid.Value.Z, 1e-12)
	// The reading is exactly Q*c: Q is orthogonal, so the stretch charge is 0.
	lin := q.Linear()
	exactY := new(big.Rat).Mul(lin[1][0], big.NewRat(2, 1))
	got := new(big.Rat)
	got.SetFloat64(g.centroid.Value.Y)
	delta := new(big.Rat).Sub(got, exactY)
	delta.Abs(delta)
	bound := new(big.Rat)
	require.NotNil(t, bound.SetFloat64(g.centroid.Bound.Base()))
	require.LessOrEqual(t, delta.Cmp(bound), 0, "reading strays from exact Q*c beyond its bound")
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
	body, err := source.placedWith(t.Context(), documentHandle(joined.n.Doc), joined.n.Doc.NextProducer, composed, q)
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

	body, err := source.placedWith(t.Context(), documentHandle(joined.n.Doc), joined.n.Doc.NextProducer, composed, q)
	require.NoError(t, err)
	exact := body.payload.(facetedPayload)
	require.Nil(t, exact.pointSection)
	require.Nil(t, exact.pointPlane)
	require.Nil(t, exact.pointSimilarity)
	require.Nil(t, exact.pointCone)
	require.Nil(t, exact.revolveLoftSector)
	require.Nil(t, exact.loftMirrorCopy)
	require.Zero(t, exact.pointPlaneCapDisplacement)

	// A nonidentity float delta moves the true solid, so it drops the point-loft
	// source while retaining the float similarity, sector, and mirror-copy records.
	body, err = source.placed(t.Context(), documentHandle(joined.n.Doc), joined.n.Doc.NextProducer, composed)
	require.NoError(t, err)
	float := body.payload.(facetedPayload)
	require.Nil(t, float.pointSection)
	require.NotNil(t, float.pointSimilarity)
	require.NotNil(t, float.revolveLoftSector)
	require.NotNil(t, float.loftMirrorCopy)

	t.Run("identity delta", func(t *testing.T) {
		require.Equal(t, r3.Identity(), source.xform)
		body, err := source.placed(t.Context(), documentHandle(joined.n.Doc), joined.n.Doc.NextProducer, r3.Identity())
		require.NoError(t, err)
		// An exact identity delta leaves the point-loft source valid.
		require.NotNil(t, body.payload.(facetedPayload).pointSection)
	})
}

// exactCopyMotionMessage is the text only the exactPlaced refusal in
// motionRun.place carries.
const exactCopyMotionMessage = "descends from an exact-rotation copy"

// verifyTranslated runs VerifyMotion with mover sliding along X through a
// static solid partner, so the pair is evaluated at every pose. The mover's
// pose is never constant, so motionRun.place is really reached.
func verifyTranslated(t *testing.T, doc *Document, mover *Body) error {
	t.Helper()
	_, err := doc.VerifyMotion(t.Context(), []*Body{mover},
		Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Millimeters(0), To: units.Millimeters(30)},
		WithResolution(units.Millimeters(100)))
	return err
}

// boxUnion is the faceted Boolean result of two boxes, one tilted, that
// ScaledAt admits.
func boxUnion(t *testing.T) *Body {
	t.Helper()
	doc := New()
	a := internalBoxBody(t, doc, -2, -2, 8, 8, 10)
	b := internalBoxBody(t, doc, -1, -1, 7, 3, 8)
	turn, err := r3.RotationAround(r3.NewVec(3, 1, 4), r3.Vec{X: 1}, units.Degrees(20))
	require.NoError(t, err)
	b, err = Transform(t.Context(), b, turn)
	require.NoError(t, err)
	u, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	return u
}

// TestCopyRotationMotionRefusalSurvivesDescendants checks that the refusal to
// re-place an exact-copy descendant during motion verification follows the
// persistent exactPlaced mark. Every case has its own live static solid
// partner and a non-constant, evaluated pair, so the guard is reached; a case
// that retires the source or earlier movers still leaves that partner.
func TestCopyRotationMotionRefusalSurvivesDescendants(t *testing.T) {
	shift, err := r3.Translation(r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	turn := func() CopyOption { return WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30)) }
	cases := []struct {
		name   string
		source func(t *testing.T) *Body // nil means mirrorJoinedLoft
		build  func(t *testing.T, doc *Document, source, exact *Body) *Body
	}{
		{"exact copy", nil, func(t *testing.T, _ *Document, _, exact *Body) *Body { return exact }},
		{"zero-angle second exact copy", nil, func(t *testing.T, _ *Document, _, exact *Body) *Body {
			b, err := Copy(t.Context(), exact, WithCopyRotation(r3.Vec{}, r3.Vec{X: 1}, units.Degrees(0)))
			require.NoError(t, err)
			return b
		}},
		{"copy with no options", nil, func(t *testing.T, _ *Document, _, exact *Body) *Body {
			b, err := Copy(t.Context(), exact)
			require.NoError(t, err)
			return b
		}},
		{"transform", nil, func(t *testing.T, _ *Document, _, exact *Body) *Body {
			b, err := Transform(t.Context(), exact, shift)
			require.NoError(t, err)
			return b
		}},
		{"retired source then transform", nil, func(t *testing.T, doc *Document, source, exact *Body) *Body {
			require.NoError(t, Remove(source))
			b, err := Transform(t.Context(), exact, shift)
			require.NoError(t, err)
			return b
		}},
		{"retired source then copy", nil, func(t *testing.T, doc *Document, source, exact *Body) *Body {
			require.NoError(t, Remove(source))
			b, err := Copy(t.Context(), exact)
			require.NoError(t, err)
			return b
		}},
		{"scaled", boxUnion, func(t *testing.T, _ *Document, _, exact *Body) *Body {
			b, err := ScaledAt(t.Context(), exact, r3.Vec{}, units.Scalar(1))
			require.NoError(t, err)
			return b
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			maker := tc.source
			if maker == nil {
				maker = mirrorJoinedLoft
			}
			source := maker(t)
			doc := documentHandle(source.n.Doc)
			exact, err := Copy(t.Context(), source, turn())
			require.NoError(t, err)
			mover := tc.build(t, doc, source, exact)
			require.True(t, mover.payload.(facetedPayload).exactPlaced)
			// A live static solid partner on the mover's path, added last so no
			// case can retire it.
			internalBoxBody(t, doc, 6, -2, 10, 2, 10)
			err = verifyTranslated(t, doc, mover)
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, exactCopyMotionMessage)
		})
	}

	t.Run("placement control", func(t *testing.T) {
		source := mirrorJoinedLoft(t)
		rot, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30))
		require.NoError(t, err)
		control, err := Copy(t.Context(), source, WithCopyPlacement(rot))
		require.NoError(t, err)
		require.False(t, control.payload.(facetedPayload).exactPlaced)
		internalBoxBody(t, documentHandle(source.n.Doc), 6, -2, 10, 2, 10)
		require.NoError(t, verifyTranslated(t, documentHandle(source.n.Doc), control))
	})
}

func TestCopyRotationExactPlacedMarkFollowsPlacementOnly(t *testing.T) {
	source := mirrorJoinedLoft(t)
	require.False(t, source.payload.(facetedPayload).exactPlaced)
	zero, err := Copy(t.Context(), source, WithCopyRotation(r3.Vec{}, r3.Vec{X: 1}, units.Degrees(0)))
	require.NoError(t, err)
	require.True(t, zero.payload.(facetedPayload).exactPlaced, "a zero angle is still an exact placement")
	again, err := Copy(t.Context(), zero)
	require.NoError(t, err)
	require.True(t, again.payload.(facetedPayload).exactPlaced)
}

// pointLoftTooth is a fitted-side point loft with its apex at the origin.
func pointLoftTooth(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), 8)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := []*sketch.Point{
		s.CreatePoint(3, -1), s.CreatePoint(4, -1.5), s.CreatePoint(5, -1),
		s.CreatePoint(5, 1), s.CreatePoint(3, 1),
	}
	for _, p := range pts {
		s.Fix(p)
	}
	_, err = s.CreateFitSpline(pts[0], pts[1], pts[2])
	require.NoError(t, err)
	for i := 2; i < len(pts); i++ {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tooth, err := doc.LoftFromPoint(t.Context(), r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	return tooth
}

// pointLoftConeTool is the full X-axis cone that exactPointCone recognizes.
func pointLoftConeTool(t *testing.T, doc *Document, apex float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := []*sketch.Point{s.CreatePoint(-2, 0), s.CreatePoint(-2, apex+2), s.CreatePoint(apex, 0)}
	for _, p := range pts {
		s.Fix(p)
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Revolve(t.Context(), doc, s, s.Profiles()[0],
		SketchLine{Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	return tool
}

// TestCopyRotationRoundTripDropsPointSource checks that a float round trip
// cannot restore the point-loft source of an exact copy. An exact rotation
// about X through (0,1,1) at 2^-60 has a nonzero exact translation, yet the
// inverse float rotation brings the accumulated placement back to the float
// identity; the dropped pointSection must keep the cone trim from matching.
func TestCopyRotationRoundTripDropsPointSource(t *testing.T) {
	doc := New()
	tooth := pointLoftTooth(t, doc)
	cone := pointLoftConeTool(t, doc, 5)
	require.NotNil(t, tooth.payload.(facetedPayload).pointSection)
	control, matched, err := tryPointConeBoolean(t.Context(), meshbool.OpCut, doc, doc.nextProducerID(), tooth, cone)
	require.NoError(t, err)
	require.True(t, matched, "the unplaced point loft takes the cone trim")
	require.NotNil(t, control.payload.(facetedPayload).pointCone)

	center, axis, angle := r3.NewVec(0, 1, 1), r3.Vec{X: 1}, units.Radians(math.Ldexp(1, -60))
	record, _, err := exactrotation.New(center, axis, angle)
	require.NoError(t, err)
	nonzero := false
	for _, v := range record.Translation() {
		nonzero = nonzero || v.Sign() != 0
	}
	require.True(t, nonzero, "the exact translation is nonzero")

	exact, err := Copy(t.Context(), tooth, WithCopyRotation(center, axis, angle))
	require.NoError(t, err)

	d, err := r3.RotationAround(center, axis, angle)
	require.NoError(t, err)
	back, err := d.Inverse()
	require.NoError(t, err)
	round, err := Transform(t.Context(), exact, back)
	require.NoError(t, err)
	rp := round.payload.(facetedPayload)
	require.Equal(t, r3.Identity(), rp.xform, "the float round trip is the identity")
	require.True(t, rp.exactPlaced)

	_, matched, err = tryPointConeBoolean(t.Context(), meshbool.OpCut, doc, doc.nextProducerID(), round, cone)
	require.NoError(t, err)
	require.False(t, matched, "the cone trim must not match a round-tripped exact copy")

	cut, err := Cut(t.Context(), round, cone)
	require.NoError(t, err)
	require.True(t, cut.payload.(facetedPayload).pointCone == nil, "the Cut result carries no pointCone")

	// The record itself is gone from the exact copy and from the round trip.
	require.True(t, exact.payload.(facetedPayload).pointSection == nil)
	require.True(t, rp.pointSection == nil)
}
