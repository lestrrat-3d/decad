package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is Body.Draft's fixture set (docs/draft-design.md §11, D1–D5 and
// the selector Walls). The closed forms are F1's and F5's, from
// extrude_taper_test.go's helpers.

// plainBox extrudes F1's untapered square.
func plainBox(t *testing.T, doc *decad.Document, dir decad.Direction) *decad.Body {
	t.Helper()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
	b, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(boxHeight), Dir: dir})
	require.NoError(t, err)
	return b
}

func capNeutral(b *decad.Body, role func(*decad.Body) decad.FeatureRef) decad.NeutralFace {
	return decad.NeutralFace{Body: b, Face: decad.Faces(decad.FaceCreatedBy(role(b)))}
}

func requireSameBody(t *testing.T, want, got *decad.Body) {
	t.Helper()
	require.Len(t, got.Faces(), len(want.Faces()))
	require.Len(t, got.Edges(), len(want.Edges()))
	require.Len(t, got.Vertices(), len(want.Vertices()))
	wv, gv := want.Vertices(), got.Vertices()
	for i := range wv {
		require.Equal(t, wv[i].Position(), gv[i].Position(), "vertex %d", i)
	}
	vw, err := want.Volume()
	require.NoError(t, err)
	vg, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, vw, vg)
	aw, err := want.Area()
	require.NoError(t, err)
	ag, err := got.Area()
	require.NoError(t, err)
	require.Equal(t, aw, ag)
	cw, err := want.Centroid()
	require.NoError(t, err)
	cg, err := got.Centroid()
	require.NoError(t, err)
	require.Equal(t, cw, cg)
}

// TestDraftEquivalence is D1: drafting the untapered box about its start cap
// builds the body a tapered Extrude of the same sketch builds, bit for bit.
// Shown to fail first: with the draft's nearStart inverted, the comparison
// with the tapered Extrude failed (Not equal on the first vertex) and
// TestDraftOtherCap's far cap sat at the wrong level.
func TestDraftEquivalence(t *testing.T) {
	t.Parallel()
	const deg = 5.0
	for _, dir := range []decad.Direction{decad.Along, decad.Against} {
		doc := decad.New()
		s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
		want := taperExtrude(t, doc, s, p, boxHeight, deg, dir)
		box := plainBox(t, doc, dir)
		neutral := capNeutral(box, decad.CapStart)
		if dir == decad.Against {
			neutral = capNeutral(box, decad.CapEnd)
		}
		got, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box)), neutral, units.Degrees(deg))
		require.NoError(t, err)
		requireManifold(t, got)
		requireSameBody(t, want, got)

		d := bfMul(bf(boxHeight), taperTan(deg))
		vol, err := got.Volume()
		require.NoError(t, err)
		requireMeasurementCovers(t, "volume", vol, boxVolume(d), taperCeiling)
	}
}

// TestDraftOtherCap is D2: the neutral face capEnd puts the near section at z1
// and the far cap, now capStart, at z0, with F1's volume.
func TestDraftOtherCap(t *testing.T) {
	t.Parallel()
	const deg = 5.0
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	got, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box)), capNeutral(box, decad.CapEnd), units.Degrees(deg))
	require.NoError(t, err)
	requireManifold(t, got)
	d := bfMul(bf(boxHeight), taperTan(deg))
	vol, err := got.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, boxVolume(d), taperCeiling)

	start, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(got))).Exactly(1).SelectFaces(got)
	require.NoError(t, err)
	far, err := start[0].Area()
	require.NoError(t, err)
	half := bfSub(bf(boxSide), bfMul(bf(2), d))
	requireMeasurementCovers(t, "far cap area", far, bfMul(half, half), taperCeiling)
	for _, e := range start[0].Edges() {
		require.Equal(t, 0.0, e.Start().Position().Value.Z)
	}
	end, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(got))).Exactly(1).SelectFaces(got)
	require.NoError(t, err)
	near, err := end[0].Area()
	require.NoError(t, err)
	require.Equal(t, boxSide*boxSide, near.Value.Base())
}

// TestDraftNegative is D3: a negative angle widens the body, F5's volume.
func TestDraftNegative(t *testing.T) {
	t.Parallel()
	const deg = -5.0
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	got, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box)), capNeutral(box, decad.CapStart), units.Degrees(deg))
	require.NoError(t, err)
	requireManifold(t, got)
	d := bfMul(bf(boxHeight), taperTan(deg))
	// F5: h(a² + 2a|d| + 4d²/3), the box volume with d = h·tan(-5 deg) < 0.
	vol, err := got.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, boxVolume(d), taperCeiling)
}

// TestDraftPlacedAndFilleted drafts a placed receiver and a filleted one.
func TestDraftPlacedAndFilleted(t *testing.T) {
	t.Parallel()
	const deg = 5.0
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	placed, err := box.Placed(t.Context(), generalMotion(t, 23, -7.5))
	require.NoError(t, err)
	got, err := placed.Draft(t.Context(), decad.Faces(decad.Walls(placed)), capNeutral(placed, decad.CapStart), units.Degrees(deg))
	require.NoError(t, err)
	requireManifold(t, got)
	d := bfMul(bf(boxHeight), taperTan(deg))
	vol, err := got.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "placed volume", vol, boxVolume(d), taperCeiling)

	// A box with one vertical edge filleted at r: the section loses
	// r²(1 − π/4) of area, and its walls (four lines and an arc, G1 joined)
	// all draft.
	box2 := plainBox(t, doc, decad.Along)
	const r = 3.0
	rounded, err := box2.Fillet(t.Context(),
		decad.Edges(decad.EndpointAt(r3.NewVec(boxSide/2, boxSide/2, 0)), decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(1),
		units.Millimeters(r))
	require.NoError(t, err)
	walls, err := decad.Faces(decad.Walls(rounded)).SelectFaces(rounded)
	require.NoError(t, err)
	require.Len(t, walls, 5)
	got2, err := rounded.Draft(t.Context(), decad.Faces(decad.Walls(rounded)), capNeutral(rounded, decad.CapStart), units.Degrees(deg))
	require.NoError(t, err)
	requireManifold(t, got2)
	require.Len(t, got2.Faces(), 7)
	vol2, err := got2.Volume()
	require.NoError(t, err)
	vstraight, err := rounded.Volume()
	require.NoError(t, err)
	require.Less(t, vol2.Value.Base(), vstraight.Value.Base())
	require.Greater(t, vol2.Value.Base(), 0.0)
}

// TestDraftRetires is D4: the receiver is retired, the result is live and the
// document holds one body.
func TestDraftRetires(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	got, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box)), capNeutral(box, decad.CapStart), units.Degrees(5))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{got}, doc.Bodies())
	_, err = box.Draft(t.Context(), decad.Faces(decad.Walls(box)), capNeutral(box, decad.CapStart), units.Degrees(5))
	require.ErrorIs(t, err, decad.ErrRetiredBody)
}

// TestWallsSelector pins Walls(b): every side(i, j) face of b's own producer,
// no cap, and nothing on a body whose producer mints no walls.
func TestWallsSelector(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	walls, err := decad.Faces(decad.Walls(box)).Exactly(4).SelectFaces(box)
	require.NoError(t, err)
	for _, f := range walls {
		require.True(t, isWall(f))
	}
	capped, err := decad.Faces(decad.Walls(box), decad.FaceCreatedBy(decad.CapStart(box))).SelectFaces(box)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	require.Empty(t, capped)

	other := plainBox(t, doc, decad.Along)
	_, err = decad.Faces(decad.Walls(other)).SelectFaces(box)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// TestDraftRefusals is D5: one row per Table SD gate Draft can reach, each
// asserting the sentinel and that the document is unchanged.
func TestDraftRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	walls := func(b *decad.Body) decad.FaceSelector { return decad.Faces(decad.Walls(b)) }
	deg5 := units.Degrees(5)

	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(10, 4))
	rev, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	cyl := cylinderBody(t, doc)
	other := plainBox(t, doc, decad.Along)

	cases := []struct {
		name string
		do   func() error
		want error
	}{
		{"SD1 angle kind", func() error {
			_, err := box.Draft(t.Context(), walls(box), capNeutral(box, decad.CapStart), units.Millimeters(5))
			return err
		}, decad.ErrUnitKind},
		{"SD2 right angle", func() error {
			_, err := box.Draft(t.Context(), walls(box), capNeutral(box, decad.CapStart), units.Degrees(90))
			return err
		}, decad.ErrDegenerate},
		{"SD18 zero angle", func() error {
			_, err := box.Draft(t.Context(), walls(box), capNeutral(box, decad.CapStart), units.Degrees(0))
			return err
		}, decad.ErrDegenerate},
		{"SD19 zero faces", func() error {
			_, err := box.Draft(t.Context(), walls(box),
				decad.NeutralFace{Body: box, Face: decad.Faces(decad.NormalTo(r3.NewVec(1, 1, 1)))}, deg5)
			return err
		}, decad.ErrCardinality},
		{"SD19 two faces", func() error {
			_, err := box.Draft(t.Context(), walls(box), decad.NeutralFace{Body: box, Face: decad.Faces(decad.Planar())}, deg5)
			return err
		}, decad.ErrCardinality},
		{"SD19 curved neutral face", func() error {
			_, err := box.Draft(t.Context(), walls(box),
				decad.NeutralFace{Body: cyl, Face: decad.Faces(decad.Cylindrical())}, deg5)
			return err
		}, decad.ErrDegenerate},
		{"SD20 wall as neutral face", func() error {
			_, err := box.Draft(t.Context(), walls(box),
				decad.NeutralFace{Body: box, Face: decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))}, deg5)
			return err
		}, decad.ErrUnsupported},
		{"SD20 another body's cap", func() error {
			_, err := box.Draft(t.Context(), walls(box), capNeutral(other, decad.CapStart), deg5)
			return err
		}, decad.ErrUnsupported},
		{"SD21 empty selection", func() error {
			_, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box), decad.Facing(r3.NewVec(0, 0, 1))),
				capNeutral(box, decad.CapStart), deg5)
			return err
		}, decad.ErrNoMatch},
		{"SD22 the other cap selected", func() error {
			_, err := box.Draft(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(box))),
				capNeutral(box, decad.CapStart), deg5)
			return err
		}, decad.ErrDegenerate},
		{"SD23 revolve", func() error {
			_, err := rev.Draft(t.Context(), decad.Faces(), capNeutral(box, decad.CapStart), deg5)
			return err
		}, decad.ErrUnsupported},
		{"empty neutral face", func() error {
			_, err := box.Draft(t.Context(), walls(box), decad.NeutralFace{}, deg5)
			return err
		}, decad.ErrDegenerate},
		{"nil selector", func() error {
			_, err := box.Draft(t.Context(), nil, capNeutral(box, decad.CapStart), deg5)
			return err
		}, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			before := doc.Bodies()
			err := tc.do()
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, before, doc.Bodies())
		})
	}

	t.Run("SD17 retired receiver and SD23 second draft", func(t *testing.T) {
		d2 := decad.New()
		b := plainBox(t, d2, decad.Along)
		got, err := b.Draft(t.Context(), walls(b), capNeutral(b, decad.CapStart), deg5)
		require.NoError(t, err)
		_, err = b.Draft(t.Context(), walls(b), capNeutral(b, decad.CapStart), deg5)
		require.ErrorIs(t, err, decad.ErrRetiredBody)
		before := d2.Bodies()
		_, err = got.Draft(t.Context(), walls(got), capNeutral(got, decad.CapStart), deg5)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, d2.Bodies())
	})
}

// cylinderBody revolves a rectangle into a cylinder about the u axis.
func cylinderBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, func(s *sketch.Sketch) {
		r := s.CreateRectangle(0, 0, 10, 4)
		s.Fix(r.A)
	})
	b, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return b
}

// TestDraftSymmetricReceiver drafts a Symmetric-extruded prism, whose sweep
// runs from z0 = -h/2 to z1 = h/2, about each cap in turn. The body is F1's
// frustum shifted to the receiver's levels: volume h(a² - 2ad + 4d²/3), area
// a² + (a - 2d)² + 4(a - d)√(h² + d²), and the far corners at ±(a/2 - d) on
// the far cap's level. Shown to fail first: with Draft's nearStart inverted, the far corners sat
// 0.87 mm from their closed-form points and the vertex check went red.
func TestDraftSymmetricReceiver(t *testing.T) {
	t.Parallel()
	const deg = 5.0
	for _, neutralStart := range []bool{true, false} {
		doc := decad.New()
		s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
		box, err := doc.Extrude(s, p, decad.Symmetric{D: units.Millimeters(boxHeight / 2)})
		require.NoError(t, err)
		bounds, err := box.Bounds()
		require.NoError(t, err)
		h := bounds.Max.Z - bounds.Min.Z
		require.Equal(t, boxHeight, h)
		require.Equal(t, -boxHeight/2, bounds.Min.Z)

		role, farZ := decad.CapEnd, -boxHeight/2
		if neutralStart {
			role, farZ = decad.CapStart, boxHeight/2
		}
		got, err := box.Draft(t.Context(), decad.Faces(decad.Walls(box)), capNeutral(box, role), units.Degrees(deg))
		require.NoError(t, err)
		requireManifold(t, got)

		d := bfMul(bf(h), taperTan(deg))
		vol, err := got.Volume()
		require.NoError(t, err)
		requireMeasurementCovers(t, "volume", vol, boxVolume(d), taperCeiling)

		A, H := bf(boxSide), bf(h)
		top := bfSub(A, bfMul(bf(2), d))
		slant := bfSqrt(bfAdd(bfMul(H, H), bfMul(d, d)))
		area := bfAdd(bfAdd(bfMul(A, A), bfMul(top, top)), bfMul(bf(4), bfSub(A, d), slant))
		ar, err := got.Area()
		require.NoError(t, err)
		requireMeasurementCovers(t, "area", ar, area, taperCeiling)

		half := bfSub(bf(boxSide/2), d)
		for _, su := range []float64{-1, 1} {
			for _, sv := range []float64{-1, 1} {
				requireVertexAt(t, got, liftExact(xyFrame(t), r3.Identity(), bfMul(bf(su), half), bfMul(bf(sv), half), bf(farZ)))
			}
		}
	}
}
