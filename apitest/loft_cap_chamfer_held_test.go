package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// outerCapLoops selects both complete outer cap loops of b.
func outerCapLoops(b *decad.Body) *decad.EdgeQuery {
	return decad.Edges(decad.OuterLoopOf(decad.CapStart(b))).Or(decad.OuterLoopOf(decad.CapEnd(b)))
}

// capAreaTotal sums the areas of b's start and end caps.
func capAreaTotal(t *testing.T, b *decad.Body) (float64, [2]float64) {
	t.Helper()
	var areas [2]float64
	for i, role := range []decad.FeatureRef{decad.CapStart(b), decad.CapEnd(b)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(b)
		require.NoError(t, err)
		area, err := faces[0].Area()
		require.NoError(t, err)
		areas[i] = area.Value.Base()
	}
	return areas[0] + areas[1], areas
}

// heldVolume is the signed volume of b's verified mesh.
func heldVolume(t *testing.T, b *decad.Body) float64 {
	t.Helper()
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	return meshVolume(mesh)
}

// requireOnChamferLevels checks that every vertex of b's verified mesh lies on
// a cap level or on a band's side level, and that the shell has Euler
// characteristic euler: 2 without a hole, 0 with one bore.
func requireOnChamferLevels(t *testing.T, b *decad.Body, height, d float64, euler int, msg string) {
	t.Helper()
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err, msg)
	require.True(t, mesh.BoundaryVerified(), msg)
	require.True(t, mesh.VolumeVerified(), msg)
	require.Equal(t, euler, loftMirrorChamferEuler(mesh), "the chamfered shell's genus: %s", msg)
	for _, p := range mesh.Vertices() {
		near := func(level float64) bool { return math.Abs(p.Z-level) <= 1e-12 }
		require.True(t, near(0) || near(d) || near(height-d) || near(height),
			"%s: vertex z=%g is off the cap and side levels", msg, p.Z)
	}
}

// TestLoftCapChamferRoundedTwistedGear chamfers both outer cap loops of a gear
// outline lofted to the same outline turned about its axis, after its root
// corners are rounded. A setback whose inset would drop a tooth tip refuses. Each band removes, to within the twist of its short
// side, the triangle of legs d along the cap outline it gives up, so the
// removed volume is d/2 times the cap area lost.
func TestLoftCapChamferRoundedTwistedGear(t *testing.T) {
	const height = 3.0
	twist := 14.5 * math.Pi / 180
	g := newTwistedGear(5, 20*math.Pi/180, 0.05)
	body, bottom, top := g.loft(t, height, twist, 0)
	source, err := decad.Fillet(t.Context(), body, rungQuery(bottom, top, height).Exactly(len(bottom)),
		units.Millimeters(0.25))
	require.NoError(t, err)
	before, err := source.Volume()
	require.NoError(t, err)
	capsBefore, _ := capAreaTotal(t, source)
	// The rounded loft's reading is its ruled solid; the chamfer denotes the
	// held solid cut by the bands, whose bound also covers the ruled one.
	held := heldVolume(t, source)
	for _, d := range []float64{0.1, 0.3} {
		rounded, err := decad.Copy(t.Context(), source)
		require.NoError(t, err)
		chamfered, err := decad.Chamfer(t.Context(), rounded, outerCapLoops(rounded), units.Millimeters(d))
		if d > 0.2 {
			// Twice the setback is wider than a tooth's tip land, so the inset
			// outline would drop the tip.
			require.ErrorIs(t, err, decad.ErrUnsupported, "setback %g", d)
			continue
		}
		require.NoError(t, err, "setback %g", d)
		after, err := chamfered.Volume()
		require.NoError(t, err)
		capsAfter, _ := capAreaTotal(t, chamfered)
		shrink := capsBefore - capsAfter
		require.Positive(t, shrink, "setback %g", d)
		require.InEpsilon(t, d/2*shrink, held-after.Value.Base(), 0.03, "setback %g", d)
		require.InDelta(t, before.Value.Base()-d/2*shrink, after.Value.Base(),
			before.Bound.Base()+after.Bound.Base()+0.03*d/2*shrink, "setback %g", d)
		require.Equal(t, len(bottom), filletRoleCount(chamfered), "the rounded roots stay: setback %g", d)
		requireOnChamferLevels(t, chamfered, height, d, 2, "gear")
	}
}

// TestLoftCapChamferSquareFrustum chamfers both outer cap loops of a loft from
// a square to a smaller square. Each wall is a plane, so every horizontal
// slice of a cap's removed region lies between two squares whose sides change
// linearly with height, and Simpson's rule gives its volume exactly. The
// inset caps are squares of side s−2d.
func TestLoftCapChamferSquareFrustum(t *testing.T) {
	const height, s0, s1, d = 6.0, 10.0, 7.0, 0.4
	w := sketch.NewWorld()
	sk0, p0, _ := squareSection(t, w, w.XY(), s0, 0, 0)
	sk1, p1, _ := squareSection(t, w, splitTopPlane(t, w, height), s1, 0, 0)
	body, err := decad.Loft(t.Context(), decad.New(), sk0, p0, sk1, p1)
	require.NoError(t, err)
	before, err := body.Volume()
	require.NoError(t, err)
	chamfered, err := decad.Chamfer(t.Context(), body, outerCapLoops(body), units.Millimeters(d))
	require.NoError(t, err)
	after, err := chamfered.Volume()
	require.NoError(t, err)

	// side(z) is the wall's square side at depth z below a cap whose side is
	// at, sloping by k per unit depth; band(z) the bevel's.
	removed := func(at, k float64) float64 {
		side := func(z float64) float64 { return at + k*z }
		band := func(z float64) float64 { return at - 2*d + (side(d)-at+2*d)*z/d }
		area := func(z float64) float64 { return side(z)*side(z) - band(z)*band(z) }
		return d / 6 * (area(0) + 4*area(d/2) + area(d))
	}
	k := (s1 - s0) / height
	want := removed(s0, k) + removed(s1, -k)
	require.InDelta(t, want, before.Value.Base()-after.Value.Base(), before.Bound.Base()+after.Bound.Base()+1e-9)
	_, caps := capAreaTotal(t, chamfered)
	require.InDelta(t, (s0-2*d)*(s0-2*d), caps[0], 1e-9)
	require.InDelta(t, (s1-2*d)*(s1-2*d), caps[1], 1e-9)
	requireOnChamferLevels(t, chamfered, height, d, 2, "frustum")
}

// TestLoftCapChamferTurnedSquare chamfers a loft from a square to the same
// square turned about the axis. Both inset caps are squares of side s−2d. A
// setback whose bands would meet refuses.
func TestLoftCapChamferTurnedSquare(t *testing.T) {
	const height, s, d = 4.0, 8.0, 0.25
	w := sketch.NewWorld()
	sk0, p0, _ := squareSection(t, w, w.XY(), s, 0, 0)
	sk1, p1, _ := squareSection(t, w, splitTopPlane(t, w, height), s, 25*math.Pi/180, 0)
	body, err := decad.Loft(t.Context(), decad.New(), sk0, p0, sk1, p1)
	require.NoError(t, err)
	_, err = decad.Chamfer(t.Context(), body, outerCapLoops(body), units.Millimeters(height/2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	chamfered, err := decad.Chamfer(t.Context(), body, outerCapLoops(body), units.Millimeters(d))
	require.NoError(t, err)
	_, caps := capAreaTotal(t, chamfered)
	for _, area := range caps {
		require.InDelta(t, (s-2*d)*(s-2*d), area, 1e-9)
	}
	requireOnChamferLevels(t, chamfered, height, d, 2, "turned square")
}

// TestLoftCapChamferKeepsTheBore chamfers both outer cap loops of a rounded
// twisted gear loft after a centered through-bore. The bore wall and its two
// rims stay whole: the result keeps one cylinder of the bore's radius, and
// every cap vertex within the bore's reach lies on it. The removed held volume
// is d/2 times the cap area lost within 3%.
func TestLoftCapChamferKeepsTheBore(t *testing.T) {
	const height, r, d = 3.0, 0.5, 0.1
	twist := 14.5 * math.Pi / 180
	g := newTwistedGear(5, 20*math.Pi/180, 0.05)
	body, bottom, top := g.loft(t, height, twist, 0)
	rounded, err := decad.Fillet(t.Context(), body, rungQuery(bottom, top, height).Exactly(len(bottom)),
		units.Millimeters(0.25))
	require.NoError(t, err)
	bored, err := decad.Cut(t.Context(), rounded, circleBodyAtZ(t, rounded.Document(), 0, r, -1, height+2))
	require.NoError(t, err)
	held := heldVolume(t, bored)
	capsBefore, _ := capAreaTotal(t, bored)
	chamfered, err := decad.Chamfer(t.Context(), bored, outerCapLoops(bored), units.Millimeters(d))
	require.NoError(t, err)
	after, err := chamfered.Volume()
	require.NoError(t, err)
	capsAfter, _ := capAreaTotal(t, chamfered)
	shrink := capsBefore - capsAfter
	require.Positive(t, shrink)
	require.InEpsilon(t, d/2*shrink, held-after.Value.Base(), 0.03)

	walls, err := decad.Faces(decad.Cylindrical()).SelectFaces(chamfered)
	require.NoError(t, err)
	require.Len(t, walls, 1)
	cylinder, ok := walls[0].Surface().(decad.Cylinder)
	require.True(t, ok)
	require.InDelta(t, r, cylinder.Radius.Base(), 1e-12)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	rim := 0
	for _, p := range mesh.Vertices() {
		if math.Hypot(p.X, p.Y) < 2*r {
			require.InDelta(t, r, math.Hypot(p.X, p.Y), 1e-9, "a vertex near the axis is on the bore")
			rim++
		}
	}
	require.Positive(t, rim)
	requireOnChamferLevels(t, chamfered, height, d, 0, "bored gear")
}
