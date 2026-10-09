package apitest_test

import (
	"math"
	"math/big"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is the subset draft's fixture set (docs/draft-design.md §10.2,
// Table RD row RD3, §14 PR 5): a selection naming some walls moves only their
// carriers, and the kept walls stay vertical. Each closed form integrates the
// section area A(z) over the sweep, with d = h·tan α evaluated in the test.

// pickWalls selects every wall of b that pick accepts, by the wall's own
// side(i, j) roles (FaceCreatedBy over each origin).
func pickWalls(t *testing.T, b *decad.Body, pick func(*decad.Face) bool) decad.FaceSelector {
	t.Helper()
	var q *decad.FaceQuery
	for _, f := range b.Faces() {
		if !isWall(f) || !pick(f) {
			continue
		}
		for _, o := range f.Origins() {
			if q == nil {
				q = decad.Faces(decad.FaceCreatedBy(o))
				continue
			}
			q.Or(decad.FaceCreatedBy(o))
		}
	}
	require.NotNil(t, q, "no wall of the body matches")
	return q
}

// hasVertexAt reports whether f has a vertex exactly at (u, v, 0).
func hasVertexAt(f *decad.Face, u, v float64) bool {
	for _, l := range f.Loops() {
		for _, e := range l.Edges() {
			if e.Start().Position().Value == r3.NewVec(u, v, 0) {
				return true
			}
		}
	}
	return false
}

// hasRole reports whether f carries the given role.
func hasRole(f *decad.Face, role string) bool {
	for _, o := range f.Origins() {
		if o.Role == role {
			return true
		}
	}
	return false
}

// TestDraftOneWall drafts the +u wall of F1's untapered box alone. The section
// at height z is [−a/2, a/2 − z·tan α] × [−a/2, a/2], so A(z) = a(a − z·tan α),
// the volume is h·a(a − d/2), the centroid sits at
// u = −(ad/2 − d²/3)/(2(a − d/2)) and z = h(a/2 − d/3)/(a − d/2), and the area
// is a² + a(a − d) + a√(h² + d²) + ah + h(2a − d). The moved wall's far corners
// sit at (a/2 − d, ±a/2, h); the kept walls keep their own far corners and
// their horizontal normals.
func TestDraftOneWall(t *testing.T) {
	t.Parallel()
	const deg = 5.0
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	got, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box), decad.Facing(r3.NewVec(1, 0, 0))),
		capNeutral(box, decad.CapStart), units.Degrees(deg))
	require.NoError(t, err)
	requireManifold(t, got)
	require.Len(t, got.Faces(), 6)
	require.Len(t, got.Edges(), 12)
	require.Len(t, got.Vertices(), 8)

	tan := taperTan(deg)
	d := bfMul(bf(boxHeight), tan)
	A, H := bf(boxSide), bf(boxHeight)
	want := bfMul(H, A, bfSub(A, bfQuo(d, bf(2))))
	vol, err := got.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, want, taperCeiling)

	slant := bfSqrt(bfAdd(bfMul(H, H), bfMul(d, d)))
	area := bfAdd(bfAdd(bfAdd(bfMul(A, A), bfMul(A, bfSub(A, d))), bfAdd(bfMul(A, slant), bfMul(A, H))), bfMul(H, bfSub(bfMul(bf(2), A), d)))
	ar, err := got.Area()
	require.NoError(t, err)
	requireMeasurementCovers(t, "area", ar, area, taperCeiling)

	c, err := got.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, c.Exactness)
	half := bfSub(A, bfQuo(d, bf(2)))
	cz := bfQuo(bfMul(H, bfSub(bfQuo(A, bf(2)), bfQuo(d, bf(3)))), half)
	requireCovers(t, "centroid z", c.Value.Z, c.Bound.Base(), cz, taperCeiling)
	cu := bfQuo(bfSub(bfQuo(bfMul(A, d), bf(2)), bfQuo(bfMul(d, d), bf(3))), bfMul(bf(-2), half))
	requireCovers(t, "centroid u", c.Value.X, c.Bound.Base(), cu, 1e-6)

	moved := bfSub(bf(boxSide/2), d)
	for _, sv := range []float64{-1, 1} {
		requireVertexAt(t, got, liftExact(xyFrame(t), r3.Identity(), moved, bf(sv*boxSide/2), H))
		requireVertexAt(t, got, liftExact(xyFrame(t), r3.Identity(), bf(-boxSide/2), bf(sv*boxSide/2), H))
	}

	cos, sin := cosSinOf(tan)
	zero, one := bf(0), bf(1)
	planes, cones := facesByKind(got)
	require.Len(t, planes, 4)
	require.Empty(t, cones)
	for _, f := range planes {
		n, err := f.NormalAt(f.Loops()[0].Edges()[0].Start().Position().Value)
		require.NoError(t, err)
		switch {
		case n.Value.X > 0.5:
			requireWallNormal(t, f, [3]*big.Float{cos, zero, sin})
		case n.Value.X < -0.5:
			requireWallNormal(t, f, [3]*big.Float{bfMul(bf(-1), one), zero, zero})
		case n.Value.Y > 0.5:
			requireWallNormal(t, f, [3]*big.Float{zero, one, zero})
		default:
			requireWallNormal(t, f, [3]*big.Float{zero, bfMul(bf(-1), one), zero})
		}
	}

	// The record re-derives the same subset under a placement.
	placed, err := got.Placed(t.Context(), generalMotion(t, 23, -7.5))
	require.NoError(t, err)
	pv, err := placed.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "placed volume", pv, want, taperCeiling)
}

// TestDraftNotchWall drafts the L section's notch wall u = 10 alone (F4's
// section). The moved wall meets the kept wall v = 10 at the reflex corner and
// the kept wall v = 20 at the top, both by the miter of a moved and an unmoved
// line, so the upper arm narrows to [0, 10 − t] and A(z) = 300 − 10·z·tan α:
// the volume is h(300 − 5d), the far reflex corner (10 − d, 10, h) and the far
// top corner (10 − d, 20, h).
func TestDraftNotchWall(t *testing.T) {
	t.Parallel()
	const h, deg = 10.0, 5.0
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawPolygon(
		[2]float64{0, 0}, [2]float64{20, 0}, [2]float64{20, 10},
		[2]float64{10, 10}, [2]float64{10, 20}, [2]float64{0, 20}))
	l, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(h)})
	require.NoError(t, err)
	notch := pickWalls(t, l, func(f *decad.Face) bool { return hasVertexAt(f, 10, 10) && hasVertexAt(f, 10, 20) })
	got, err := l.Draft(t.Context(), notch, capNeutral(l, decad.CapStart), units.Degrees(deg))
	require.NoError(t, err)
	requireManifold(t, got)
	require.Len(t, got.Faces(), 8)

	d := bfMul(bf(h), taperTan(deg))
	want := bfMul(bf(h), bfSub(bf(300), bfMul(bf(5), d)))
	vol, err := got.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, want, taperCeiling)
	u := bfSub(bf(10), d)
	requireVertexAt(t, got, liftExact(xyFrame(t), r3.Identity(), u, bf(10), bf(h)))
	requireVertexAt(t, got, liftExact(xyFrame(t), r3.Identity(), u, bf(20), bf(h)))
	requireVertexAt(t, got, liftExact(xyFrame(t), r3.Identity(), bf(20), bf(10), bf(h)))
}

// TestDraftHoleAlone drafts F7's hole wall alone: the hole widens to r + t
// while the outer wall stays a vertical Cylinder of radius R, so
// A(z) = π(R² − (r + z·tan α)²) and the volume is πh(R² − r² − rd − d²/3).
func TestDraftHoleAlone(t *testing.T) {
	t.Parallel()
	const ro, ri, h, deg = 10.0, 4.0, 10.0, 5.0
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 1, drawRing(ro, ri))
	ring, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(h)})
	require.NoError(t, err)
	hole := pickWalls(t, ring, func(f *decad.Face) bool { return hasRole(f, "side(1,0)") })
	got, err := ring.Draft(t.Context(), hole, capNeutral(ring, decad.CapStart), units.Degrees(deg))
	require.NoError(t, err)
	requireManifold(t, got)

	d := bfMul(bf(h), taperTan(deg))
	R, r := bf(ro), bf(ri)
	area := bfSub(bfSub(bfSub(bfMul(R, R), bfMul(r, r)), bfMul(r, d)), bfQuo(bfMul(d, d), bf(3)))
	vol, err := got.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, bfMul(bigPi, bf(h), area), taperCeiling)

	var outer, inner *decad.Face
	for _, f := range got.Faces() {
		switch {
		case hasRole(f, "side(0,0)"):
			outer = f
		case hasRole(f, "side(1,0)"):
			inner = f
		}
	}
	require.NotNil(t, outer)
	require.NotNil(t, inner)
	cyl, ok := outer.Surface().(decad.Cylinder)
	require.True(t, ok, "the kept outer wall is a Cylinder, got %T", outer.Surface())
	require.Equal(t, ro, cyl.Radius.Base())
	require.IsType(t, decad.Cone{}, inner.Surface())
	for _, e := range outer.Edges() {
		require.Equal(t, ro, e.Curve().(decad.Circle3).Radius.Base(), "both rims of the kept wall keep its radius")
	}
	for _, e := range inner.Edges() {
		c := e.Curve().(decad.Circle3)
		if e.Start().Position().Value.Z == 0 {
			require.Equal(t, ri, c.Radius.Base())
			continue
		}
		require.Greater(t, c.Radius.Base(), ri, "the hole opens toward the far end")
	}
}

// TestDraftSubsetChargesTheTangentSpan pins the offset amount's span at a
// corner between a moved and a kept wall. A 2 mm square with its +u wall
// drafted holds its far corners at (a/2 − d, ±a/2, h), where a/2 − d is a
// Sterbenz subtraction, so each held corner is the exact miter of the held d
// and its whole displacement is the gap between the held d and the d the taper
// denotes. Shown to fail first: with capcontour.AmountsDisplacement reading
// each moved walk's span as the point amount (its width zeroed), both far
// corners of the moved wall sat 1.06e-16 from their exact points, outside
// their bound, and the vertex check went red.
func TestDraftSubsetChargesTheTangentSpan(t *testing.T) {
	t.Parallel()
	const a, h, deg = 2.0, 10.0, 5.0
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, a))
	box, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(h)})
	require.NoError(t, err)
	got, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box), decad.Facing(r3.NewVec(1, 0, 0))),
		capNeutral(box, decad.CapStart), units.Degrees(deg))
	require.NoError(t, err)
	u := bfSub(bf(a/2), bfMul(bf(h), taperTan(deg)))
	for _, sv := range []float64{-1, 1} {
		requireVertexAt(t, got, liftExact(xyFrame(t), r3.Identity(), u, bf(sv*a/2), bf(h)))
	}
}

// TestDraftSubsetVerify is DD5 and DD6 over the subset fixtures: each verifies
// valid, and the tolerance gate reads a diameter for each.
func TestDraftSubsetVerify(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	one, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box), decad.Facing(r3.NewVec(1, 0, 0))),
		capNeutral(box, decad.CapStart), units.Degrees(5))
	require.NoError(t, err)
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 1, drawRing(12, 5))
	ring, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Against})
	require.NoError(t, err)
	hole, err := ring.Draft(t.Context(), pickWalls(t, ring, func(f *decad.Face) bool { return hasRole(f, "side(1,0)") }),
		capNeutral(ring, decad.CapEnd), units.Degrees(5))
	require.NoError(t, err)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	for _, diag := range report.Diagnostics {
		require.NotEqual(t, decad.DiagToleranceReferenceUnavailable, diag.Code, "%+v", diag)
	}
	for _, b := range []*decad.Body{one, hole} {
		br, err := report.ForBody(b)
		require.NoError(t, err)
		require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
	}
}

// TestDraftSubsetRefusals is the mixed-corner rule (docs/draft-design.md
// §10.2): a straight wall of F3's slot drafted alone meets both semicircles at
// G1 joins whose two walls would move differently, which is SD4; and a wall
// selection that matches nothing is the selector's ErrNoMatch. Each leaves the
// document unchanged. Shown to fail first: with offset2d.SharpCornerJoin's
// mixed-amount refusal removed, the slot's corner took the G1 foot of the
// kept wall and refused as SD13 instead, and the message check went red.
func TestDraftSubsetRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSlot(0, 0))
	slot, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(8)})
	require.NoError(t, err)
	box := plainBox(t, doc, decad.Along)

	cases := []struct {
		name string
		do   func() error
		want error
		text string
	}{
		{"SD4 one straight wall of a slot", func() error {
			_, err := slot.Draft(t.Context(), decad.Faces(decad.Walls(slot), decad.Facing(r3.NewVec(0, 1, 0))),
				capNeutral(slot, decad.CapStart), units.Degrees(3))
			return err
		}, decad.ErrUnsupported, "select both walls or neither (draft SD4)"},
		{"SD21 empty selection", func() error {
			_, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box), decad.Facing(r3.NewVec(0, 0, 1))),
				capNeutral(box, decad.CapStart), units.Degrees(5))
			return err
		}, decad.ErrNoMatch, ""},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			before := doc.Bodies()
			err := tc.do()
			require.ErrorIs(t, err, tc.want)
			require.True(t, strings.Contains(err.Error(), tc.text), "%v", err)
			require.Equal(t, before, doc.Bodies())
		})
	}

	// Both straight walls together still meet the semicircles at mixed G1
	// joins; the whole slot drafts as F3 does.
	_, err = slot.Draft(t.Context(), decad.Faces(decad.Walls(slot), decad.Planar()), capNeutral(slot, decad.CapStart), units.Degrees(3))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = slot.Draft(t.Context(), decad.Faces(decad.Walls(slot)), capNeutral(slot, decad.CapStart), units.Degrees(3))
	require.NoError(t, err)
}

// TestDraftSubsetMesh meshes the subset fixtures (Table DD row DD1 over
// docs/draft-design.md §10.2) at three chord tolerances: each mesh is closed,
// proves its volume, attributes every facet to a face of the body, and
// publishes a positive Bound no larger than the tapered extrude of the same
// section publishes (F1 for the box, F7 for the ring), since a kept wall only
// removes taper. For F7's hole drafted alone, no sample of the kept outer
// Cylinder or of the hole's Cone sits farther from the mesh than Bound, and
// Bound covers the outer near ring's own sagitta. Shown to fail first: with
// the band's per-walk setbacks dropped (the kept outer wall's cap radius
// rounding read against the draft's d), the ring's Bound rose to 1.09, 0.92
// and 0.88 mm against F7's 0.38, 0.11 and 0.037.
func TestDraftSubsetMesh(t *testing.T) {
	t.Parallel()
	const ro, ri, h, deg = 10.0, 4.0, 10.0, 5.0
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	one, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box), decad.Facing(r3.NewVec(1, 0, 0))),
		capNeutral(box, decad.CapStart), units.Degrees(deg))
	require.NoError(t, err)
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 1, drawRing(ro, ri))
	ring, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(h)})
	require.NoError(t, err)
	hole, err := ring.Draft(t.Context(), pickWalls(t, ring, func(f *decad.Face) bool { return hasRole(f, "side(1,0)") }),
		capNeutral(ring, decad.CapStart), units.Degrees(deg))
	require.NoError(t, err)
	sq, sp := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
	f1 := taperExtrude(t, doc, sq, sp, boxHeight, deg, decad.Along)
	rs, rp := taperSketch(t, r3.NewVec(0, 0, 0), 1, drawRing(ro, ri))
	f7 := taperExtrude(t, doc, rs, rp, h, deg, decad.Along)
	d, _ := bfMul(bf(h), taperTan(deg)).Float64()
	walls := []func(s, th float64) r3.Vec{draftConeWall(0, ro, ro, h), draftConeWall(0, ri, ri+d, h)}

	for _, pair := range [][2]*decad.Body{{one, f1}, {hole, f7}} {
		b, tapered := pair[0], pair[1]
		faces := map[*decad.Face]struct{}{}
		for _, f := range b.Faces() {
			faces[f] = struct{}{}
		}
		for _, tol := range draftMeshTolerances {
			mesh, err := b.Tessellate(t.Context(), units.Millimeters(tol))
			require.NoError(t, err, "tol %g", tol)
			requireWatertight(t, mesh)
			require.True(t, mesh.VolumeVerified(), "tol %g", tol)
			for _, f := range mesh.SourceFaces() {
				_, ok := faces[f]
				require.True(t, ok, "tol %g: a facet names a face the body does not hold", tol)
			}
			ref, err := tapered.Tessellate(t.Context(), units.Millimeters(tol))
			require.NoError(t, err)
			bound := mesh.Bound().Mag()
			require.Positive(t, bound, "tol %g", tol)
			require.LessOrEqual(t, bound, ref.Bound().Mag(), "tol %g: Bound above the tapered extrude's", tol)
			if b != hole {
				continue
			}
			require.LessOrEqual(t, nearRingSagitta(mesh, ro), bound, "tol %g", tol)
			for _, wall := range walls {
				for i := range 9 {
					for j := range 37 {
						p := wall(float64(i)/8, 2*math.Pi*float64(j)/37)
						require.LessOrEqual(t, distanceToMesh(mesh, p), bound,
							"tol %g: true wall sample %v is farther from the mesh than Bound", tol, p)
					}
				}
			}
		}
	}
}

// TestDraftSubsetUndercut is DD7 over a subset draft: F1's box with its +u
// wall drafted alone. Along −e the drafted wall reads −sin 5° and is listed
// as an undercut; along +e it reads sin 5° and clears. Either way the three
// kept walls stand parallel to the pull, their normal component zero within
// its bound, so the survey leaves them undecided rather than clearing a wall
// with no draft.
func TestDraftSubsetUndercut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	got, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box), decad.Facing(r3.NewVec(1, 0, 0))),
		capNeutral(box, decad.CapStart), units.Degrees(5))
	require.NoError(t, err)
	var moved *decad.Face
	for _, f := range draftWalls(got) {
		n, err := f.NormalAt(f.Loops()[0].Edges()[0].Start().Position().Value)
		require.NoError(t, err)
		if n.Value.X > 0.5 {
			moved = f
		}
	}
	require.NotNil(t, moved)

	report, br := draftSurvey(t, doc, got, r3.NewVec(0, 0, -1))
	require.Equal(t, []*decad.Face{moved}, br.Undercut.Faces)
	require.True(t, hasDiagnostic(report, decad.DiagUndercut))
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))

	report, br = draftSurvey(t, doc, got, r3.NewVec(0, 0, 1))
	require.NotContains(t, br.Undercut.Faces, moved)
	require.False(t, hasDiagnostic(report, decad.DiagUndercut))
	require.True(t, hasDiagnostic(report, decad.DiagUndecidedUndercut))
}
