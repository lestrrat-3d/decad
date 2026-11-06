package apitest_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// twistedGear is a whole involute gear outline: per tooth a radial line from
// the root circle to the base circle, a fitted involute flank, a tip arc, the
// other flank and line, and a root arc to the next tooth. Its section turned by
// a twist is the same outline with every point rotated about the origin.
type twistedGear struct {
	teeth                  int
	pressure, lash         float64
	samples                int
	pitch, base, root, tip float64
}

func newTwistedGear(teeth int, pressure, lash float64) twistedGear {
	n := float64(teeth)
	return twistedGear{teeth: teeth, pressure: pressure, lash: lash, samples: 7,
		pitch: n / 2, base: n / 2 * math.Cos(pressure), root: (n - 2.5) / 2, tip: (n + 2) / 2}
}

// flanks samples one tooth's left and right involute flanks, base to tip,
// rotated by angle.
func (g twistedGear) flanks(angle float64) ([]gearPoint, []gearPoint) {
	mirrored := func(r float64) gearPoint {
		p := gearInvolute(g.base, r)
		return gearPoint{p.x, -p.y}
	}
	atPitch := mirrored(g.pitch)
	shift := math.Pi/(2*float64(g.teeth)) - g.lash/(2*g.pitch) - math.Atan2(atPitch.y, atPitch.x)
	left := make([]gearPoint, g.samples)
	right := make([]gearPoint, g.samples)
	for i := range g.samples {
		r := g.base + (g.tip-g.base)*float64(i)/float64(g.samples-1)
		l := gearRotate(mirrored(r), shift)
		left[i] = gearRotate(l, angle)
		right[i] = gearRotate(gearPoint{l.x, -l.y}, angle)
	}
	return left, right
}

// section draws the outline turned by twist on plane. A positive radius draws
// it with every root corner already rounded by a tangent arc of that radius,
// placed from the closed-form tangent circle of a radial line and the root
// circle. It returns each tooth's right and left root corners of the sharp
// outline.
func (g twistedGear) section(t *testing.T, w *sketch.World, plane *sketch.Plane, twist,
	radius float64) (*sketch.Sketch, *sketch.Profile, []gearPoint) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.Origin()
	fixed := func(p gearPoint) *sketch.Point {
		pt := s.CreatePoint(p.x, p.y)
		s.Fix(pt)
		return pt
	}
	unit := func(p gearPoint) gearPoint {
		l := math.Hypot(p.x, p.y)
		return gearPoint{p.x / l, p.y / l}
	}
	var corners []gearPoint
	var first, previous *sketch.Point
	for k := range g.teeth {
		left, right := g.flanks(2*math.Pi*float64(k)/float64(g.teeth) + twist)
		lp := make([]*sketch.Point, g.samples)
		rp := make([]*sketch.Point, g.samples)
		for i := range g.samples {
			lp[i], rp[i] = fixed(left[i]), fixed(right[i])
		}
		rightCorner := gearPoint{right[0].x * g.root / g.base, right[0].y * g.root / g.base}
		leftCorner := gearPoint{left[0].x * g.root / g.base, left[0].y * g.root / g.base}
		corners = append(corners, rightCorner, leftCorner)
		rightRoot, rightLine := rightCorner, rightCorner
		leftRoot, leftLine := leftCorner, leftCorner
		var rightCenter, leftCenter gearPoint
		if radius > 0 {
			rightCenter, rightRoot, rightLine = gearRootBlend(unit(right[0]), g.root, radius, -1)
			leftCenter, leftRoot, leftLine = gearRootBlend(unit(left[0]), g.root, radius, +1)
		}
		rr, lr := fixed(rightRoot), fixed(leftRoot)
		if previous != nil {
			s.CreateArc(center, previous, rr)
		} else {
			first = rr
		}
		if radius > 0 {
			rl := fixed(rightLine)
			s.CreateArc(fixed(rightCenter), rl, rr)
			s.CreateLine(rl, rp[0])
		} else {
			s.CreateLine(rr, rp[0])
		}
		_, err := s.CreateFitSpline(rp...)
		require.NoError(t, err)
		s.CreateArc(center, rp[g.samples-1], lp[g.samples-1])
		_, err = s.CreateFitSpline(lp...)
		require.NoError(t, err)
		if radius > 0 {
			ll := fixed(leftLine)
			s.CreateLine(lp[0], ll)
			s.CreateArc(fixed(leftCenter), lr, ll)
		} else {
			s.CreateLine(lp[0], lr)
		}
		previous = lr
	}
	s.CreateArc(center, previous, first)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	require.True(t, s.Profiles()[0].Valid)
	return s, s.Profiles()[0], corners
}

// loft lofts the outline from the XY plane to the outline turned by twist on
// the plane height above it.
func (g twistedGear) loft(t *testing.T, height, twist, radius float64) (*decad.Body, []gearPoint, []gearPoint) {
	t.Helper()
	w := sketch.NewWorld()
	s0, p0, bottom := g.section(t, w, w.XY(), 0, radius)
	s1, p1, top := g.section(t, w, splitTopPlane(t, w, height), twist, radius)
	body, err := decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1)
	require.NoError(t, err)
	return body, bottom, top
}

// rungQuery selects each rung from a bottom corner to the matching top corner.
func rungQuery(bottom, top []gearPoint, height float64) *decad.EdgeQuery {
	var query *decad.EdgeQuery
	for i := range bottom {
		lower := r3.NewVec(bottom[i].x, bottom[i].y, 0)
		upper := r3.NewVec(top[i].x, top[i].y, height)
		if query == nil {
			query = decad.Edges(decad.EndpointAt(lower), decad.EndpointAt(upper))
			continue
		}
		query.Or(decad.EndpointAt(lower), decad.EndpointAt(upper))
	}
	return query
}

// filletRoleCount counts the distinct fillet connector roles on a body's faces.
func filletRoleCount(body *decad.Body) int {
	roles := map[string]struct{}{}
	for _, face := range body.Faces() {
		for _, origin := range face.Origins() {
			if strings.HasPrefix(origin.Role, "fillet(0,") {
				roles[origin.Role] = struct{}{}
			}
		}
	}
	return len(roles)
}

// requireVerifiedSolid checks that body is one solid whose tessellation proves
// its boundary and volume.
func requireVerifiedSolid(t *testing.T, body *decad.Body, msg string) {
	t.Helper()
	require.True(t, body.IsSolid(), msg)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err, msg)
	require.True(t, mesh.BoundaryVerified(), msg)
	require.True(t, mesh.VolumeVerified(), msg)
}

// TestLoftFilletTwistedGearRoots rounds every root corner of a loft from a
// gear outline to the same outline turned about its axis, the section pair of
// a helical gear. Each section receives its own tangent arcs, so the result
// must match, within both volume bounds, a loft of the two outlines drawn
// already rounded from the closed-form tangent circle.
func TestLoftFilletTwistedGearRoots(t *testing.T) {
	for _, tc := range []struct {
		name                  string
		teeth                 int
		pressure, lash, twist float64
		height, radius        float64
	}{
		{name: "five teeth", teeth: 5, pressure: 20 * math.Pi / 180, twist: 14.5 * math.Pi / 180, height: 5, radius: 0.3},
		{name: "seven teeth backlash", teeth: 7, pressure: 25 * math.Pi / 180, lash: 0.05,
			twist: -10 * math.Pi / 180, height: 4, radius: 0.35},
		{name: "six teeth steep", teeth: 6, pressure: 20 * math.Pi / 180, lash: 0.05,
			twist: 30 * math.Pi / 180, height: 3, radius: 0.25},
	} {
		g := newTwistedGear(tc.teeth, tc.pressure, tc.lash)
		body, bottom, top := g.loft(t, tc.height, tc.twist, 0)
		before, err := body.Volume()
		require.NoError(t, err, tc.name)
		rounded, err := decad.Fillet(t.Context(), body, rungQuery(bottom, top, tc.height).Exactly(len(bottom)),
			units.Millimeters(tc.radius))
		require.NoError(t, err, tc.name)
		after, err := rounded.Volume()
		require.NoError(t, err, tc.name)
		require.Greater(t, after.Value.Base(), before.Value.Base()+before.Bound.Base()+after.Bound.Base(), tc.name)
		require.Equal(t, len(bottom), filletRoleCount(rounded), tc.name)
		_, err = rungQuery(bottom, top, tc.height).SelectEdges(rounded)
		require.ErrorIs(t, err, decad.ErrNoMatch, "every sharp root rung is gone: %s", tc.name)

		reference, _, _ := g.loft(t, tc.height, tc.twist, tc.radius)
		want, err := reference.Volume()
		require.NoError(t, err, tc.name)
		require.InDelta(t, want.Value.Base(), after.Value.Base(), want.Bound.Base()+after.Bound.Base(), tc.name)
		requireVerifiedSolid(t, rounded, tc.name)
	}
}

// TestLoftFilletTwistedGearMirrorJoin rounds the root corners of a twisted
// gear loft, mirrors it across its end section and joins the two halves, the
// construction of a herringbone gear. The joined solid's volume is twice the
// rounded half's, and twice the independently drawn rounded half's. The
// outline is sized so the sewn shell's box-overlapping candidate pairs exceed
// the unstructured crossing audit's ceiling (15 teeth of this outline already
// do), so the join must decide its two outer caps by the banded cap proof.
func TestLoftFilletTwistedGearMirrorJoin(t *testing.T) {
	const height, radius = 2.5, 0.3
	twist := 14.5 * math.Pi / 180
	g := newTwistedGear(16, 20*math.Pi/180, 0.05)
	g.samples = 15
	body, bottom, top := g.loft(t, height, twist, 0)
	half, err := decad.Fillet(t.Context(), body, rungQuery(bottom, top, height).Exactly(len(bottom)),
		units.Millimeters(radius))
	require.NoError(t, err)
	end, err := r3.NewFrame(r3.Vec{Z: height}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	mirrored, err := decad.MirroredCopy(t.Context(), half, decad.MirrorFrame{Frame: end})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), half, mirrored)
	require.NoError(t, err)
	got, err := joined.Volume()
	require.NoError(t, err)
	halfVolume, err := half.Volume()
	require.NoError(t, err)
	require.InDelta(t, 2*halfVolume.Value.Base(), got.Value.Base(), got.Bound.Base()+2*halfVolume.Bound.Base())
	reference, _, _ := g.loft(t, height, twist, radius)
	want, err := reference.Volume()
	require.NoError(t, err)
	require.InDelta(t, 2*want.Value.Base(), got.Value.Base(), got.Bound.Base()+2*want.Bound.Base())
	requireVerifiedSolid(t, joined, "mirror join")
}

// squareSection draws a square of side size centered on the origin and turned
// by twist. A positive radius rounds its corner at (-size/2, -size/2) before
// the turn with a quarter arc of that radius. It returns that corner.
func squareSection(t *testing.T, w *sketch.World, plane *sketch.Plane, size, twist,
	radius float64) (*sketch.Sketch, *sketch.Profile, gearPoint) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	h := size / 2
	fixed := func(x, y float64) *sketch.Point {
		p := gearRotate(gearPoint{x, y}, twist)
		pt := s.CreatePoint(p.x, p.y)
		s.Fix(pt)
		return pt
	}
	if radius == 0 {
		corners := []*sketch.Point{fixed(-h, -h), fixed(h, -h), fixed(h, h), fixed(-h, h)}
		for i := range corners {
			s.CreateLine(corners[i], corners[(i+1)%len(corners)])
		}
	} else {
		start, end := fixed(-h+radius, -h), fixed(-h, -h+radius)
		corners := []*sketch.Point{start, fixed(h, -h), fixed(h, h), fixed(-h, h), end}
		for i := range len(corners) - 1 {
			s.CreateLine(corners[i], corners[i+1])
		}
		s.CreateArc(fixed(-h+radius, -h+radius), end, start)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0], gearRotate(gearPoint{-h, -h}, twist)
}

// TestLoftFilletSectionsWithDifferentRecords rounds one corner of a loft whose
// two square sections differ, turned or smaller, and checks the result against
// a loft of the two squares drawn already rounded. A radius that fits the
// first section and consumes a side of the second refuses.
func TestLoftFilletSectionsWithDifferentRecords(t *testing.T) {
	const height = 6.0
	build := func(size1, twist, radius float64) (*decad.Body, gearPoint, gearPoint) {
		w := sketch.NewWorld()
		s0, p0, c0 := squareSection(t, w, w.XY(), 10, 0, radius)
		s1, p1, c1 := squareSection(t, w, splitTopPlane(t, w, height), size1, twist, radius)
		body, err := decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1)
		require.NoError(t, err)
		return body, c0, c1
	}
	for _, tc := range []struct {
		name                 string
		size1, twist, radius float64
	}{
		{name: "turned", size1: 10, twist: 20 * math.Pi / 180, radius: 1.5},
		{name: "smaller", size1: 6, radius: 1},
		{name: "turned and smaller", size1: 7, twist: -35 * math.Pi / 180, radius: 2},
	} {
		body, c0, c1 := build(tc.size1, tc.twist, 0)
		query := rungQuery([]gearPoint{c0}, []gearPoint{c1}, height).Exactly(1)
		rounded, err := decad.Fillet(t.Context(), body, query, units.Millimeters(tc.radius))
		require.NoError(t, err, tc.name)
		got, err := rounded.Volume()
		require.NoError(t, err, tc.name)
		reference, _, _ := build(tc.size1, tc.twist, tc.radius)
		want, err := reference.Volume()
		require.NoError(t, err, tc.name)
		require.InDelta(t, want.Value.Base(), got.Value.Base(), want.Bound.Base()+got.Bound.Base()+1e-9, tc.name)
		require.Equal(t, 1, filletRoleCount(rounded), tc.name)
		requireVerifiedSolid(t, rounded, tc.name)
	}

	body, c0, c1 := build(6, 0, 0)
	query := rungQuery([]gearPoint{c0}, []gearPoint{c1}, height).Exactly(1)
	_, err := decad.Fillet(t.Context(), body, query, units.Millimeters(6.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
