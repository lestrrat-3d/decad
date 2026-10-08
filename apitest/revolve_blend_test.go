package apitest_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers docs/modify-reach-design.md §7 (Table RX row RX2):
// Fillet and Chamfer on a revolve's swept meridian junctions. Every fixture is
// drawn on the XY plane and revolved about the sketch's u axis, so a meridian
// point (u, v) is the axis coordinate z = u and the radius ρ = v, and the
// world x axis is the revolve axis.
//
// Legs shown to fail before these fixtures were accepted: matching a selected
// edge by radius alone, without the junction circle's centre and axis, sends
// the torus, partial-turn, cone, chord-kind and placement tests red (a
// same-radius rim maps to the wrong corner); skipping the axis re-resolution
// of the rewritten meridian lets the spindle subtest build; and dropping the
// payload's blend descriptors from the revolve build leaves no blend role, so
// every blendFaces reader goes red. Measurement bounds are the existing
// revolve build's, unchanged here; each Measures call asserts that bound
// encloses the closed form.

// meridianSketch draws the closed polygon pts with every vertex fixed and
// returns its one profile.
func meridianSketch(t *testing.T, pts [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	ps := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		ps[i] = s.CreatePoint(p[0], p[1])
		s.Fix(ps[i])
	}
	for i := range ps {
		s.CreateLine(ps[i], ps[(i+1)%len(ps)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// revolveMeridian revolves the polygon pts about the u axis.
func revolveMeridian(t *testing.T, doc *decad.Document, pts [][2]float64, a decad.AngularExtent) *decad.Body {
	t.Helper()
	s, p := meridianSketch(t, pts)
	body, err := doc.Revolve(s, p, uAxis, a)
	require.NoError(t, err)
	return body
}

// shaftMeridian is a turned shaft with one shoulder: a radius-10 journal over
// z ∈ [0, 10] stepping down to a radius-5 journal over z ∈ [10, 20]. Its
// junctions off the axis are the two outer rims (20, 5) and (0, 10), the
// shoulder's outer edge (10, 10), all convex, and the shoulder's inner corner
// (10, 5), the one concave junction.
var shaftMeridian = [][2]float64{{0, 0}, {20, 0}, {20, 5}, {10, 5}, {10, 10}, {0, 10}}

// shaftQ and shaftMzr are the unblended shaft's ∫ρ dA and ∫zρ dA over its
// meridian: two rectangles, each the product of its own one-dimensional
// moments.
const (
	shaftQ   = 10*50.0 + 10*12.5
	shaftMzr = 50*50.0 + 150*12.5
)

// spandrel holds the three closed-form moments of the region a radius-r
// fillet moves at a right-angle corner, in the corner's own local frame (x and
// y measured from the corner along the two walls into the blend): area
// (1 − π/4)r², first moments k·r·area each with k = (10 − 3π)/(12 − 3π), and
// ∫xy dA = (19/24 − π/4)r⁴.
func spandrel(r float64) (float64, float64, float64) {
	area := (1 - math.Pi/4) * r * r
	k := (10 - 3*math.Pi) / (12 - 3*math.Pi)
	return area, k * r * area, (19.0/24 - math.Pi/4) * r * r * r * r
}

// blendFaces returns the faces carrying a kind(i,j) role, each checked to
// carry a side(i,j) role naming the same (loop, segment).
func blendFaces(t *testing.T, body *decad.Body, kind string) []*decad.Face {
	t.Helper()
	var out []*decad.Face
	for _, f := range body.Faces() {
		var blend, side []string
		for _, o := range f.Origins() {
			switch {
			case strings.HasPrefix(o.Role, kind+"("):
				blend = append(blend, strings.TrimPrefix(o.Role, kind))
			case strings.HasPrefix(o.Role, "side("):
				side = append(side, strings.TrimPrefix(o.Role, "side"))
			}
		}
		if len(blend) == 0 {
			continue
		}
		require.Equal(t, side, blend, `a blend wall's %s role names the same (loop, segment) as its side role`, kind)
		out = append(out, f)
	}
	return out
}

func concaveJunctions() *decad.EdgeQuery {
	return decad.Edges(decad.Circular(), decad.Concave())
}

func TestRevolveFilletShoulderTorus(t *testing.T) {
	t.Parallel()
	const r = 2.0
	doc := decad.New()
	shaft := revolveMeridian(t, doc, shaftMeridian, decad.FullRevolution{})
	picked, err := concaveJunctions().SelectEdges(shaft)
	require.NoError(t, err)
	require.Len(t, picked, 1, `the shoulder's inner corner is the one concave junction`)
	_, isCircle := picked[0].Curve().(decad.Circle3)
	require.True(t, isCircle, `a full turn sweeps the junction to a latitude circle`)

	filleted, err := shaft.Fillet(t.Context(), concaveJunctions(), units.Millimeters(r))
	require.NoError(t, err)
	require.True(t, filleted.IsSolid())
	requireManifold(t, filleted)
	require.Equal(t, []*decad.Body{filleted}, doc.Bodies(), `the fillet retires its receiver`)

	// The concave fillet fills the corner's spandrel in. In the corner frame
	// x = z − 10, y = ρ − 5, so the meridian's moments grow by the spandrel's
	// own, shifted to the corner, and Pappus turns them into volume and the
	// axial centroid.
	a, m, ixy := spandrel(r)
	q := shaftQ + 5*a + m
	mzr := shaftMzr + 50*a + 10*m + 5*m + ixy
	decadtest.MeasuresVolume(t, filleted, units.CubicMillimeters(2*math.Pi*q))
	decadtest.MeasuresCentroid(t, filleted, r3.NewVec(mzr/q, 0, 0))

	// Area: the shoulder annulus loses the ring ρ ∈ [5, 7], the small journal
	// loses 2 mm of its length, and the torus patch adds 2π·∫ρ ds over its
	// quarter arc, 2π(7π − 4).
	wantArea := 500*math.Pi - math.Pi*(49-25) - 2*math.Pi*5*r + 2*math.Pi*(7*math.Pi-4)
	decadtest.MeasuresArea(t, filleted, units.SquareMillimeters(wantArea))

	// The blend wall is a torus whose tube is the fillet radius exactly, its
	// major radius the arc centre's ρ = 5 + r, centred on the axis at z = 10 + r.
	blends := blendFaces(t, filleted, "fillet")
	require.Len(t, blends, 1)
	torus, ok := blends[0].Surface().(decad.Torus)
	require.True(t, ok, `an off-axis fillet arc sweeps a torus`)
	require.Equal(t, r, torus.Minor.Mag(), `the tube radius is the fillet radius exactly`)
	require.InDelta(t, 5+r, torus.Major.Mag(), 1e-12)
	require.InDelta(t, 10+r, torus.Center.X, 1e-12)
	require.Zero(t, torus.Center.Y)
	require.Zero(t, torus.Center.Z)
	faceArea, err := blends[0].Area()
	require.NoError(t, err)
	decadtest.Measures(t, `torus patch area`, faceArea, units.SquareMillimeters(2*math.Pi*(7*math.Pi-4)))

	// The concave fillet is a concave feature of radius r, and the revolve's
	// own meridian survey reads it.
	rep := decadtest.Verify(t, doc, decad.WithConcaveRadius())
	decadtest.MeasuresConcaveRadius(t, rep.Bodies[0], units.Millimeters(r))
}

func TestRevolveFilletPartialTurnArc(t *testing.T) {
	t.Parallel()
	// A quarter turn sweeps the same junction to an Arc3, and filleting it
	// rewrites the same meridian: the torus is the full turn's, and the volume
	// is a quarter of it.
	const r = 2.0
	full, err := revolveMeridian(t, decad.New(), shaftMeridian, decad.FullRevolution{}).
		Fillet(t.Context(), concaveJunctions(), units.Millimeters(r))
	require.NoError(t, err)

	doc := decad.New()
	quarter := revolveMeridian(t, doc, shaftMeridian, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	picked, err := concaveJunctions().SelectEdges(quarter)
	require.NoError(t, err)
	require.Len(t, picked, 1)
	_, isArc := picked[0].Curve().(decad.Arc3)
	require.True(t, isArc, `a partial turn sweeps the junction to an arc`)

	filleted, err := quarter.Fillet(t.Context(), concaveJunctions(), units.Millimeters(r))
	require.NoError(t, err)
	requireManifold(t, filleted)

	a, m, _ := spandrel(r)
	q := shaftQ + 5*a + m
	decadtest.MeasuresVolume(t, filleted, units.CubicMillimeters(math.Pi/2*q))

	fullTorus := blendFaces(t, full, "fillet")[0].Surface().(decad.Torus)
	blends := blendFaces(t, filleted, "fillet")
	require.Len(t, blends, 1)
	require.Equal(t, fullTorus, blends[0].Surface(), `the partial turn rewrites the same meridian`)

	// Both caps carry the rewritten meridian: each cap's area is the meridian
	// area plus the spandrel.
	for _, ref := range []decad.FeatureRef{decad.CapStart(filleted), decad.CapEnd(filleted)} {
		caps, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(filleted)
		require.NoError(t, err)
		capArea, err := caps[0].Area()
		require.NoError(t, err)
		decadtest.Measures(t, `cap area`, capArea, units.SquareMillimeters(150+a))
	}
}

func TestRevolveFilletSphere(t *testing.T) {
	t.Parallel()
	// A shaft with a pointed end: the cone tip from (10, 5) to (14, 0) meets
	// the journal ρ = 5 at a convex junction. A radius-5 fillet there has its
	// centre on the axis — at distance 5 from the journal ρ = 5 and from the
	// cone's line — so the blend sweeps a sphere: a ball-nosed end.
	const r = 5.0
	pts := [][2]float64{{0, 0}, {14, 0}, {10, 5}, {0, 5}}
	doc := decad.New()
	body := revolveMeridian(t, doc, pts, decad.FullRevolution{})
	nose := decad.Edges(decad.Circular(), decad.EndpointAt(r3.NewVec(10, 5, 0)))
	filleted, err := body.Fillet(t.Context(), nose, units.Millimeters(r))
	require.NoError(t, err)
	requireManifold(t, filleted)

	blends := blendFaces(t, filleted, "fillet")
	require.Len(t, blends, 1)
	sphere, ok := blends[0].Surface().(decad.Sphere)
	require.True(t, ok, `a fillet arc centred on the axis sweeps a sphere`)
	require.InDelta(t, r, sphere.Radius.Mag(), 1e-12, `the sphere radius is the fillet radius`)
	cz := 14 - math.Sqrt(41)
	require.InDelta(t, cz, sphere.Center.X, 1e-12)

	// The rewritten meridian, walked counter-clockwise: the axis run, the cone
	// line up to its tangent foot, the fillet arc about (cz, 0) from the
	// cone's foot to the journal's foot (cz, 5), the journal back to (0, 5).
	foot := [2]float64{cz + r*5/math.Sqrt(41), r * 4 / math.Sqrt(41)}
	q, mzr := meridianMoments([]meridianPiece{
		lineMeridianPiece([2]float64{0, 0}, [2]float64{14, 0}),
		lineMeridianPiece([2]float64{14, 0}, foot),
		arcMeridianPiece([2]float64{cz, 0}, r, math.Atan2(4, 5), math.Pi/2),
		lineMeridianPiece([2]float64{cz, 5}, [2]float64{0, 5}),
		lineMeridianPiece([2]float64{0, 5}, [2]float64{0, 0}),
	})
	decadtest.MeasuresVolume(t, filleted, units.CubicMillimeters(2*math.Pi*q))
	decadtest.MeasuresCentroid(t, filleted, r3.NewVec(mzr/q, 0, 0))
}

func TestRevolveChamferChordKinds(t *testing.T) {
	t.Parallel()
	// A diamond ring: the square with corners (10, 2), (12, 4), (10, 6) and
	// (8, 4) revolved a full turn. An equal chamfer cuts each right-angle
	// corner with a chord perpendicular to the corner's bisector, so the two
	// corners on the axis-parallel bisector get radial chords — planes — and
	// the two on the radial bisector get axial chords — cylinders.
	const d = 1.0
	doc := decad.New()
	ring := revolveMeridian(t, doc, [][2]float64{{10, 2}, {12, 4}, {10, 6}, {8, 4}}, decad.FullRevolution{})
	chamfered, err := ring.Chamfer(t.Context(), decad.Edges(decad.Circular()).Exactly(4), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	var planes, cylinders []float64
	for _, f := range blendFaces(t, chamfered, "chamfer") {
		switch s := f.Surface().(type) {
		case decad.Plane:
			n := s.Frame.N()
			require.InDelta(t, 1, math.Abs(n.X), 1e-12, `a radial chord sweeps a plane normal to the axis`)
			planes = append(planes, s.Frame.Origin().X)
		case decad.Cylinder:
			cylinders = append(cylinders, s.Radius.Mag())
		default:
			t.Fatalf(`unexpected chamfer surface %T`, s)
		}
	}
	h := d / math.Sqrt2
	require.ElementsMatch(t, roundAll([]float64{8 + h, 12 - h}), roundAll(planes))
	require.ElementsMatch(t, roundAll([]float64{2 + h, 6 - h}), roundAll(cylinders))

	// Each corner loses a right isosceles triangle of legs d. The two at
	// ρ = 4 have their centroid at ρ = 4, the other two at 6 − 2h/3 and
	// 2 + 2h/3, which sum to 8: 2π·(d²/2)·16 removed from 2π·8·4.
	decadtest.MeasuresVolume(t, chamfered, units.CubicMillimeters(64*math.Pi-16*math.Pi*d*d))
}

// roundAll rounds each value to 1e-12 so ElementsMatch compares the closed
// forms rather than the last ulp of a computed foot.
func roundAll(vs []float64) []float64 {
	out := make([]float64, len(vs))
	for i, v := range vs {
		out[i] = math.Round(v*1e12) / 1e12
	}
	return out
}

func TestRevolveChamferShoulderCone(t *testing.T) {
	t.Parallel()
	// The shoulder's inner corner chamfered by 2: the chord from (10, 7) to
	// (12, 5) runs at 45° to the axis and sweeps a cone.
	const d = 2.0
	doc := decad.New()
	shaft := revolveMeridian(t, doc, shaftMeridian, decad.FullRevolution{})
	chamfered, err := shaft.Chamfer(t.Context(), concaveJunctions(), units.Millimeters(d))
	require.NoError(t, err)
	requireManifold(t, chamfered)

	blends := blendFaces(t, chamfered, "chamfer")
	require.Len(t, blends, 1)
	cone, ok := blends[0].Surface().(decad.Cone)
	require.True(t, ok, `an inclined chord sweeps a cone`)
	require.InDelta(t, math.Pi/4, cone.HalfAngle.Mag(), 1e-12)
	require.InDelta(t, 17, cone.Origin.X, 1e-12, `the chord's line meets the axis at z = 17`)

	// The added triangle has legs d at the corner (10, 5): area d²/2 and
	// centroid ρ = 5 + d/3, z = 10 + d/3.
	area := d * d / 2
	q := shaftQ + area*(5+d/3)
	mzr := shaftMzr + area*meanProduct(10, 5, d)
	decadtest.MeasuresVolume(t, chamfered, units.CubicMillimeters(2*math.Pi*q))
	decadtest.MeasuresCentroid(t, chamfered, r3.NewVec(mzr/q, 0, 0))
}

// meanProduct is the mean of zρ over the right isosceles triangle with legs d
// along +z and +ρ from the corner (z0, ρ0): z0ρ0 + (z0+ρ0)d/3 + d²/12.
func meanProduct(z0, rho0, d float64) float64 {
	return z0*rho0 + (z0+rho0)*d/3 + d*d/12
}

func TestRevolveBlendNonJunctionRefused(t *testing.T) {
	t.Parallel()
	t.Run(`cap and axis lines`, func(t *testing.T) {
		// A quarter turn's cap copies of the straight walls, and the one edge
		// the two caps share along the axis, are Line3 edges no junction swept.
		doc := decad.New()
		quarter := revolveMeridian(t, doc, shaftMeridian, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		sel := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)))
		_, err := quarter.Fillet(t.Context(), sel, units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		requireReasonLeads(t, err, `a fillet of a revolve edge that is not a swept meridian junction`)
		_, err = quarter.Chamfer(t.Context(), sel, units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		requireReasonLeads(t, err, `a chamfer of a revolve edge that is not a swept meridian junction`)
		require.Equal(t, []*decad.Body{quarter}, doc.Bodies(), `the refusal leaves the receiver live`)
	})
	t.Run(`cap arc`, func(t *testing.T) {
		// A filleted quarter turn's caps carry the fillet arc's own Arc3 copies.
		// They are circular like the junction arcs, and still no junction's.
		doc := decad.New()
		quarter := revolveMeridian(t, doc, shaftMeridian, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		filleted, err := quarter.Fillet(t.Context(), concaveJunctions(), units.Millimeters(2))
		require.NoError(t, err)
		capArcs := decad.Edges(decad.Circular(), decad.EndpointAt(r3.NewVec(10, 7, 0)))
		picked, err := capArcs.SelectEdges(filleted)
		require.NoError(t, err)
		require.NotEmpty(t, picked)
		_, err = filleted.Chamfer(t.Context(), capArcs, units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, `not a swept meridian junction`)
		require.Equal(t, []*decad.Body{filleted}, doc.Bodies())
	})
}

func TestRevolveBlendBaseGates(t *testing.T) {
	t.Parallel()
	t.Run(`overrun renders closed circles`, func(t *testing.T) {
		// Radius 6 at all four corners of the 10 × 10 annular section claims
		// every wall from both ends: base S6, with each matched latitude circle
		// rendered as the closed circle it is.
		ring := revolvedRing(t)
		_, err := ring.Fillet(t.Context(), decad.Edges(decad.Circular()), units.Millimeters(6))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		requireReasonLeads(t, err, `is consumed by its corner setbacks`)
		require.ErrorContains(t, err,
			`selected edge[0] closed circle through (0,5,0), centre (0,0,0), radius 5 mm`)
		require.NotContains(t, err.Error(), `to (0,5,0)`)
		_, err = revolvedRing(t).Chamfer(t.Context(), decad.Edges(decad.Circular()), units.Millimeters(6))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		requireReasonLeads(t, err, `is consumed by its corner setbacks`)
	})
	t.Run(`smooth junction`, func(t *testing.T) {
		// A filleted shaft's fillet arc meets the shoulder face tangentially:
		// the junction between them is still an edge, and no corner — base S4.
		filleted, err := revolveMeridian(t, decad.New(), shaftMeridian, decad.FullRevolution{}).
			Fillet(t.Context(), concaveJunctions(), units.Millimeters(2))
		require.NoError(t, err)
		smooth := decad.Edges(decad.Circular(), decad.EndpointAt(r3.NewVec(10, 7, 0))).Exactly(1)
		_, err = filleted.Fillet(t.Context(), smooth, units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, `there is no corner to round`)
	})
}

// notchMeridian is a solid shaft of radius 4 with a V-notch bored in from the
// axis: the boundary leaves the axis at z = 4, rises to the notch apex (5, 1)
// and returns to the axis at z = 6. The apex is the one concave junction, and
// its fillet's centre sits at ρ = 1 − r√2, below the axis once r > 1/√2.
var notchMeridian = [][2]float64{{0, 0}, {4, 0}, {5, 1}, {6, 0}, {10, 0}, {10, 4}, {0, 4}}

func TestRevolveFilletAxisGates(t *testing.T) {
	t.Parallel()
	t.Run(`builds above the axis`, func(t *testing.T) {
		filleted, err := revolveMeridian(t, decad.New(), notchMeridian, decad.FullRevolution{}).
			Fillet(t.Context(), concaveJunctions(), units.Millimeters(0.5))
		require.NoError(t, err)
		blends := blendFaces(t, filleted, "fillet")
		require.Len(t, blends, 1)
		torus, ok := blends[0].Surface().(decad.Torus)
		require.True(t, ok)
		require.InDelta(t, 1-0.5*math.Sqrt2, torus.Major.Mag(), 1e-12)
	})
	t.Run(`spindle branch`, func(t *testing.T) {
		// r = 1 keeps the rewritten meridian on the axis's own side — its
		// lowest points are the tangent feet at ρ = 1 − 1/√2 — but centres the
		// arc across the axis: a spindle torus, staged exactly as Revolve stages
		// a recorded arc centred there.
		doc := decad.New()
		body := revolveMeridian(t, doc, notchMeridian, decad.FullRevolution{})
		_, err := body.Fillet(t.Context(), concaveJunctions(), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, `spindle torus`)
		require.Equal(t, []*decad.Body{body}, doc.Bodies(), `the refusal leaves the receiver live`)
	})
}

func TestRevolveBlendRolesSurvivePlacementAndReplay(t *testing.T) {
	t.Parallel()
	const r = 2.0
	build := func(t *testing.T) *decad.Body {
		t.Helper()
		body, err := revolveMeridian(t, decad.New(), shaftMeridian, decad.FullRevolution{}).
			Fillet(t.Context(), concaveJunctions(), units.Millimeters(r))
		require.NoError(t, err)
		return body
	}
	roles := func(body *decad.Body) []string {
		var out []string
		for _, f := range body.Faces() {
			for _, o := range f.Origins() {
				out = append(out, o.Role)
			}
		}
		return out
	}

	// Replay: the same model built again mints the same roles and the same
	// blend surface.
	first, second := build(t), build(t)
	require.Equal(t, roles(first), roles(second))
	require.Equal(t, blendFaces(t, first, "fillet")[0].Surface(), blendFaces(t, second, "fillet")[0].Surface())
	v1, err := first.Volume()
	require.NoError(t, err)
	v2, err := second.Volume()
	require.NoError(t, err)
	require.Equal(t, v1, v2)

	// Placement re-evaluates the rewritten payload: the roles come through,
	// and the torus moves with the body.
	move, err := r3.Translation(r3.NewVec(5, -3, 7))
	require.NoError(t, err)
	placed, err := second.Placed(t.Context(), move)
	require.NoError(t, err)
	require.Equal(t, roles(first), roles(placed))
	torus := blendFaces(t, placed, "fillet")[0].Surface().(decad.Torus)
	require.InDelta(t, 10+r+5, torus.Center.X, 1e-12)
	require.InDelta(t, -3, torus.Center.Y, 1e-12)
	require.InDelta(t, 7, torus.Center.Z, 1e-12)
	decadtest.MeasuresVolume(t, placed, v1.Value)

	// The selector resolves against a placed receiver too: filleting the
	// placed shaft gives the placed fillet.
	doc := decad.New()
	shaft := revolveMeridian(t, doc, shaftMeridian, decad.FullRevolution{})
	movedShaft, err := shaft.Placed(t.Context(), move)
	require.NoError(t, err)
	movedFillet, err := movedShaft.Fillet(t.Context(), concaveJunctions(), units.Millimeters(r))
	require.NoError(t, err)
	require.Equal(t, roles(first), roles(movedFillet))
	require.Equal(t, torus, blendFaces(t, movedFillet, "fillet")[0].Surface())
	decadtest.MeasuresVolume(t, movedFillet, v1.Value)
}

func TestRevolveFilletSurveys(t *testing.T) {
	t.Parallel()
	// Table DX rows DX7–DX9: the result is an ordinary revolvePayload, so the
	// revolve's own surveys answer it outright. The thinnest material is the
	// small journal's diameter, 10, which the shoulder fillet does not touch.
	// Pulled along +x every face either clears or is antiparallel; pulled along
	// −x the fillet torus, whose normals sweep from +x to +ρ, is the one face
	// that opposes the pull at a point without being antiparallel.
	doc := decad.New()
	filleted, err := revolveMeridian(t, doc, shaftMeridian, decad.FullRevolution{}).
		Fillet(t.Context(), concaveJunctions(), units.Millimeters(2))
	require.NoError(t, err)
	rep := decadtest.Verify(t, doc, decad.WithMinWallThickness(units.Millimeters(1)),
		decad.WithPullDirection(r3.NewVec(1, 0, 0)), decad.WithConcaveRadius())
	require.Equal(t, decad.Sound, rep.Status)
	decadtest.MeasuresWallMinimum(t, rep.Bodies[0], units.Millimeters(10))
	decadtest.FindUndercutFaces(t, rep.Bodies[0], 0)
	decadtest.MeasuresConcaveRadius(t, rep.Bodies[0], units.Millimeters(2))

	rep = decadtest.Verify(t, doc, decad.WithPullDirection(r3.NewVec(-1, 0, 0)))
	faces := decadtest.FindUndercutFaces(t, rep.Bodies[0], 1)
	require.Equal(t, blendFaces(t, filleted, "fillet"), faces)
}

func TestRevolveFilletTessellates(t *testing.T) {
	t.Parallel()
	// Table DX row DX3: the result is an ordinary revolvePayload, so the
	// revolve tessellator meshes it, watertight, within its own bound.
	filleted, err := revolveMeridian(t, decad.New(), shaftMeridian, decad.FullRevolution{}).
		Fillet(t.Context(), concaveJunctions(), units.Millimeters(2))
	require.NoError(t, err)
	mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	vol, err := filleted.Volume()
	require.NoError(t, err)
	want := vol.Value.Mag()
	require.InEpsilon(t, want, meshVolume(mesh), 0.01)
}

// meridianPiece is one counter-clockwise boundary piece of a meridian region
// in (z, ρ), carrying its own contributions to ∫ρ dA and ∫zρ dA by Green's
// theorem: ∫ρ dA = ∮ −ρ²/2 dz and ∫zρ dA = ∮ −zρ²/2 dz.
type meridianPiece struct {
	q, mzr float64
}

// lineMeridianPiece integrates a straight piece exactly: both integrands are
// polynomials of degree at most 3 in the line parameter, which three-point
// Gauss–Legendre integrates exactly.
func lineMeridianPiece(a, b [2]float64) meridianPiece {
	nodes := [3]float64{-math.Sqrt(0.6), 0, math.Sqrt(0.6)}
	weights := [3]float64{5.0 / 9, 8.0 / 9, 5.0 / 9}
	dz := b[0] - a[0]
	var p meridianPiece
	for i, x := range nodes {
		s := (x + 1) / 2
		z := a[0] + s*dz
		rho := a[1] + s*(b[1]-a[1])
		w := weights[i] / 2
		p.q += w * (-rho * rho / 2 * dz)
		p.mzr += w * (-z * rho * rho / 2 * dz)
	}
	return p
}

// arcMeridianPiece integrates the arc about c of radius r from angle th0 to
// th1 (counter-clockwise when th1 > th0) in closed form, with z = c0 + r cos θ
// and ρ = c1 + r sin θ.
func arcMeridianPiece(c [2]float64, r, th0, th1 float64) meridianPiece {
	// ∫ −ρ²/2 dz = (r/2) ∫ ρ² sin θ dθ, and ∫ −zρ²/2 dz = (r/2) ∫ zρ² sin θ dθ;
	// expand both into monomials sinᵐθ cosⁿθ and integrate each exactly.
	sinPow := func(m int, th float64) float64 { return math.Pow(math.Sin(th), float64(m)) }
	cosPow := func(n int, th float64) float64 { return math.Pow(math.Cos(th), float64(n)) }
	// I(m, n) = ∫ sinᵐ cosⁿ dθ over [th0, th1] for the five monomials used.
	integral := func(m, n int) float64 {
		at := func(th float64) float64 {
			switch {
			case m == 1 && n == 0:
				return -math.Cos(th)
			case m == 2 && n == 0:
				return th/2 - math.Sin(2*th)/4
			case m == 3 && n == 0:
				return -math.Cos(th) + cosPow(3, th)/3
			case m == 1 && n == 1:
				return sinPow(2, th) / 2
			case m == 2 && n == 1:
				return sinPow(3, th) / 3
			case m == 3 && n == 1:
				return sinPow(4, th) / 4
			}
			panic(`unsupported monomial`)
		}
		return at(th1) - at(th0)
	}
	c0, c1 := c[0], c[1]
	// ρ² sin θ = c1² sin θ + 2c1 r sin²θ + r² sin³θ
	rho2sin := c1*c1*integral(1, 0) + 2*c1*r*integral(2, 0) + r*r*integral(3, 0)
	// zρ² sin θ = c0·ρ² sin θ + r cos θ·ρ² sin θ
	rho2sincos := c1*c1*integral(1, 1) + 2*c1*r*integral(2, 1) + r*r*integral(3, 1)
	return meridianPiece{
		q:   r / 2 * rho2sin,
		mzr: r / 2 * (c0*rho2sin + r*rho2sincos),
	}
}

// meridianMoments sums the pieces of a closed counter-clockwise boundary.
func meridianMoments(pieces []meridianPiece) (float64, float64) {
	var q, mzr float64
	for _, p := range pieces {
		q += p.q
		mzr += p.mzr
	}
	return q, mzr
}
