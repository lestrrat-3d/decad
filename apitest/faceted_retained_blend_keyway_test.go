package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The keyway rows of agent-docs/retained-edge-blend-design.md §9: a keyway
// cut into a turned shaft ends inside it, so the shaft's cylinder runs on past
// each end of the keyway's top edges. Each top edge is convex, between the
// keyway's wall (a plane) and the shaft (a cylinder); at an end inside the
// shaft the keyway's end wall is convex at the corner and takes the blend's
// end cap, and the shaft keeps its surface past the end wall. The keyway tool
// is placed by a Transform, which keeps the Cut on the mesh path.

// keywayR is the shaft's radius, keywayW half the keyway's width, keywayX0
// the keyway's end inside the shaft at the low x, and keywayRho the fillet's
// radius.
const (
	keywayR   = 10.0
	keywayW   = 3.0
	keywayX0  = 20.0
	keywayRho = 0.5
)

// keywayShaft is the shaft r = 10 along x from 0 to 80, less the keyway
// |y| ≤ 3, z ≥ 6.5 from x = x0 to x = x1.
func keywayShaft(t *testing.T, x0, x1 float64) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, 80, keywayR)
	s.Fix(r.A)
	s.Fix(r.C)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	shaft, err := decad.Revolve(t.Context(), sc.doc, s, s.Profiles()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	tool := placed(t, rectBox(t, sc.w, sc.doc, x0, -keywayW, x1, keywayW, 6), 6.5)
	cut, err := decad.Cut(t.Context(), shaft, tool)
	require.NoError(t, err)
	return cut
}

// keywayEdges selects the top edges of a keyway in a cylinder of radius
// rad whose end at x = x0 lies on the sides in ys. The mesh Boolean holds the
// ends as floats near the exact corners, so each is matched at its held
// position.
func keywayEdges(t *testing.T, b *decad.Body, rad, x0 float64, ys ...float64) *decad.EdgeQuery {
	t.Helper()
	var q *decad.EdgeQuery
	for _, y := range ys {
		p := r3.NewVec(x0, y, math.Sqrt(rad*rad-y*y))
		var at *r3.Vec
		for _, v := range b.Vertices() {
			if pos := v.Position().Value; pos.Sub(p).Len() < 5e-3 {
				at = &pos
				break
			}
		}
		require.NotNil(t, at, "no vertex near %v", p)
		preds := []decad.EdgePredicate{decad.EndpointAt(*at), decad.ParallelTo(r3.NewVec(1, 0, 0))}
		if q == nil {
			q = decad.Edges(preds...)
			continue
		}
		q = q.Or(preds...)
	}
	return q.Exactly(len(ys))
}

// keywayInside is a point (y, z) 0.125 mm or more inside the corner region of
// the r = 0.5 fillet and the d = 0.5 chamfer.
var keywayInside = [2]float64{3.15, 9.35}

// keywaySection is the cross-section of the blend of the corner K = (3, √91)
// in (y, z) between the wall y = 3 (material at y > 3) and the shaft's circle:
// the area of the corner region the blend removes, the setback down the wall,
// the angle of the shaft's setback about the axis, the blend face's width
// across the section, and the circle's radius.
type keywaySection struct {
	corner, wall, theta, width, radius float64
}

// shaftSegment is the area between the shaft's circle and its chord over an
// angle a.
func shaftSegment(a float64) float64 { return keywayR * keywayR / 2 * (a - math.Sin(a)) }

// keywayFillet is the section of a fillet of radius r: the ball lies inside
// the shaft, touching the wall at ta and the circle at tb, with its centre c
// at distance R − r from the axis. The region is the quadrilateral K, ta, c,
// tb less the ball's sector between ta and tb, plus the shaft's segment
// between tb and K.
func keywayFillet(r float64) keywaySection {
	cy := keywayW + r
	cz := math.Sqrt((keywayR-r)*(keywayR-r) - cy*cy)
	k := [2]float64{keywayW, math.Sqrt(keywayR*keywayR - keywayW*keywayW)}
	ta := [2]float64{keywayW, cz}
	scale := keywayR / (keywayR - r)
	tb := [2]float64{cy * scale, cz * scale}
	phi := math.Acos(((ta[0]-cy)*(tb[0]-cy) + (ta[1]-cz)*(tb[1]-cz)) / (r * r))
	theta := math.Acos((k[0]*tb[0] + k[1]*tb[1]) / (keywayR * keywayR))
	quad := shoelaceArea([][2]float64{k, ta, {cy, cz}, tb})
	return keywaySection{corner: quad - r*r*phi/2 + shaftSegment(theta), wall: k[1] - cz, theta: theta, width: r * phi,
		radius: keywayR}
}

// keywayChamfer is the section of a chamfer of distance d: d down the wall and
// an arc of length d round the shaft.
func keywayChamfer(d float64) keywaySection {
	alpha := math.Asin(keywayW / keywayR)
	k := [2]float64{keywayW, keywayR * math.Cos(alpha)}
	ta := [2]float64{keywayW, k[1] - d}
	tb := [2]float64{keywayR * math.Sin(alpha+d/keywayR), keywayR * math.Cos(alpha+d/keywayR)}
	return keywaySection{corner: shoelaceArea([][2]float64{k, ta, tb}) + shaftSegment(d/keywayR), wall: d,
		theta: d / keywayR, width: math.Hypot(tb[0]-ta[0], tb[1]-ta[1]), radius: keywayR}
}

// requireKeywayBlend checks a blend of edges top edges, each l long: the
// volume falls by the corner region over that length, and the area changes by
// the blend face less the wall's strip and the cylinder's band, plus a cap at
// each of the caps edge ends inside the cylinder, less a corner at each of the
// others.
func requireKeywayBlend(t *testing.T, before, after *decad.Body, sec keywaySection, edges, l float64, caps, others int) {
	t.Helper()
	v0, err := before.Volume()
	require.NoError(t, err)
	v1, err := after.Volume()
	require.NoError(t, err)
	requireVolumeChange(t, v0, v1, edges*l*sec.corner)
	a0, err := before.Area()
	require.NoError(t, err)
	a1, err := after.Area()
	require.NoError(t, err)
	strip := edges * l * (sec.width - sec.wall - sec.radius*sec.theta)
	requireAreaChange(t, a0, a1, strip+float64(caps-others)*sec.corner)
}

// requireEndFaces checks, for each plane x = const that caps names, that the
// point (x, ±at[0], at[1]) inside the corner region of each blended edge on
// side ys lies on a face in that plane when caps says the face takes the cap,
// and at least 0.1 mm off every such face when it gives up the corner.
func requireEndFaces(t *testing.T, b *decad.Body, ys []float64, at [2]float64, caps map[float64]bool) {
	t.Helper()
	for x, takes := range caps {
		for _, y := range ys {
			p := r3.NewVec(x, math.Copysign(at[0], y), at[1])
			best := decad.Measurement{}
			found := false
			for _, f := range b.Faces() {
				pl, ok := f.Surface().(decad.Plane)
				if !ok || math.Abs(math.Abs(pl.Frame.N().X)-1) > 1e-12 || math.Abs(pl.Frame.Origin().X-x) > 1e-9 {
					continue
				}
				d, err := f.DistanceToPoint(t.Context(), p, units.Millimeters(0.01))
				require.NoError(t, err)
				if !found || d.Value.Mag() < best.Value.Mag() {
					best, found = d, true
				}
			}
			require.True(t, found, "no face in the plane x = %g", x)
			t.Logf("end plane x = %g: %v lies %.3g ± %.3g from it", x, p, best.Value.Mag(), best.Bound.Mag())
			if takes {
				require.LessOrEqual(t, best.Value.Mag(), best.Bound.Mag())
				continue
			}
			require.Greater(t, best.Value.Mag()-best.Bound.Mag(), 0.1)
		}
	}
}

func TestFacetedRetainedBlendKeyway(t *testing.T) {
	t.Parallel()

	t.Run("the corner region", func(t *testing.T) {
		t.Parallel()
		// The fillet's ball at (3.5, √78) touches the wall 0.70763 mm below the
		// corner; the chamfer's section is the triangle with the shaft's
		// segment. Both regions, sampled as polygons, agree with the closed
		// forms the other subtests use.
		for _, c := range []struct {
			sec  keywaySection
			poly [][2]float64
		}{
			{keywayFillet(keywayRho), keywayRegion(0.5, true)},
			{keywayChamfer(0.5), keywayRegion(0.5, false)},
			{keywayFillet(0.3), keywayRegion(0.3, true)},
			{keywayFillet(0.1), keywayRegion(0.1, true)},
		} {
			require.InDelta(t, shoelaceArea(c.poly), c.sec.corner, 1e-7)
		}
		require.InDelta(t, 0.117968684, keywayFillet(keywayRho).corner, 1e-9)
		require.InDelta(t, 0.040674574, keywayFillet(0.3).corner, 1e-9)
		require.InDelta(t, 0.004336328, keywayFillet(0.1).corner, 1e-9)
	})

	t.Run("fillet of both top edges", func(t *testing.T) {
		t.Parallel()
		// Both ends of each edge lie inside the shaft: each end wall takes a
		// cap, 40 mm apart.
		const r = keywayRho
		body := keywayShaft(t, keywayX0, 60)
		filleted, err := decad.Fillet(t.Context(), body, keywayEdges(t, body, keywayR, keywayX0, keywayW, -keywayW),
			units.Millimeters(r))
		require.NoError(t, err)
		sec := keywayFillet(keywayRho)
		requireKeywayBlend(t, body, filleted, sec, 2, 40, 4, 0)
		requireEndFaces(t, filleted, []float64{keywayW, -keywayW}, keywayInside, map[float64]bool{20: true, 60: true})
		cylinders := 0
		for _, f := range filleted.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Radius.Base()-r) < 1e-12 {
				cylinders++
				require.InDelta(t, 1, math.Abs(c.Axis.X), 1e-12)
				require.InDelta(t, keywayW+r, math.Abs(c.Origin.Y), 1e-9)
				require.InDelta(t, math.Sqrt(78), c.Origin.Z, 1e-9)
			}
		}
		require.Equal(t, 2, cylinders)
		mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("fillet of a keyway running off the shaft's end", func(t *testing.T) {
		t.Parallel()
		// The keyway runs out through the shaft's end at x = 80, where the
		// shaft ends with the edge and its end face gives up the corner; the
		// end wall at x = 20 takes the cap.
		const r = keywayRho
		body := keywayShaft(t, keywayX0, 90)
		filleted, err := decad.Fillet(t.Context(), body, keywayEdges(t, body, keywayR, keywayX0, keywayW, -keywayW),
			units.Millimeters(r))
		require.NoError(t, err)
		sec := keywayFillet(keywayRho)
		requireKeywayBlend(t, body, filleted, sec, 2, 60, 2, 2)
		requireEndFaces(t, filleted, []float64{keywayW, -keywayW}, keywayInside, map[float64]bool{20: true, 80: false})
	})

	t.Run("chamfer of a top edge", func(t *testing.T) {
		t.Parallel()
		const d = 0.5
		body := keywayShaft(t, keywayX0, 60)
		chamfered, err := decad.Chamfer(t.Context(), body, keywayEdges(t, body, keywayR, keywayX0, keywayW),
			units.Millimeters(d))
		require.NoError(t, err)
		sec := keywayChamfer(d)
		requireKeywayBlend(t, body, chamfered, sec, 1, 40, 2, 0)
		requireEndFaces(t, chamfered, []float64{keywayW}, keywayInside, map[float64]bool{20: true, 60: true})
	})

	t.Run("fillet of a small radius", func(t *testing.T) {
		t.Parallel()
		// r = 0.3 on a keyway from x = 20.37 to 60.
		const r, x0 = 0.3, 20.37
		body := keywayShaft(t, x0, 60)
		filleted, err := decad.Fillet(t.Context(), body, keywayEdges(t, body, keywayR, x0, keywayW, -keywayW),
			units.Millimeters(r))
		require.NoError(t, err)
		requireKeywayBlend(t, body, filleted, keywayFillet(r), 2, 60-x0, 4, 0)
	})

	t.Run("fillet of a small radius running off the shaft's end", func(t *testing.T) {
		t.Parallel()
		// The keyway from x = 20.37 runs out through x = 80. Where the end wall
		// at x = 20.37 takes the cap, the ball touches the shaft's circle at tb
		// closer to it, by the profile's own last row, than the shaft's facets
		// lie inside it: the rows leave a wider last gap there (§5.0). The
		// shaft's end x = 80 gives the corner up.
		const x0 = 20.37
		for _, c := range []struct {
			r  float64
			at [2]float64 // a point 0.05 (r = 0.3) or 0.01 mm (r = 0.1) inside the corner region
		}{{0.3, [2]float64{3.05, 9.45}}, {0.1, [2]float64{3.01, 9.52}}} {
			body := keywayShaft(t, x0, 90)
			filleted, err := decad.Fillet(t.Context(), body, keywayEdges(t, body, keywayR, x0, keywayW, -keywayW),
				units.Millimeters(c.r))
			require.NoError(t, err)
			requireKeywayBlend(t, body, filleted, keywayFillet(c.r), 2, 80-x0, 2, 2)
			requireEndFaces(t, filleted, []float64{keywayW, -keywayW}, c.at, map[float64]bool{x0: true})
			mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			rep, err := filleted.Document().Verify(t.Context())
			require.NoError(t, err)
			br, err := rep.ForBody(filleted)
			require.NoError(t, err)
			require.Equal(t, decad.Sound, br.Status)
		}
	})

	t.Run("a cross hole past the keyway's end", func(t *testing.T) {
		t.Parallel()
		// The hole's rim has a vertex on the fillet's setback line at x = 70.3,
		// 10 mm past the end wall x = 60. The band ends at that end wall, so the
		// vertex is not read against the setback line and the volume falls by
		// the keyway's corners alone.
		body := keywayShaft(t, keywayX0, 60)
		holed, err := decad.Cut(t.Context(), body, keywayCrossHole(t, body, 70))
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), holed, keywayEdges(t, holed, keywayR, keywayX0, keywayW, -keywayW),
			units.Millimeters(keywayRho))
		require.NoError(t, err)
		requireKeywayBlend(t, holed, filleted, keywayFillet(keywayRho), 2, 40, 4, 0)
	})

	t.Run("an internal keyway in a hub", func(t *testing.T) {
		t.Parallel()
		// The hub's bore r = 5 holds its material outside the cylinder. The
		// keyway |y| ≤ 2 from x = 10 to 30 ends inside the hub, which runs on
		// past each end wall, and each end wall takes a cap.
		body := hubKeyway(t)
		filleted, err := decad.Fillet(t.Context(), body, keywayEdges(t, body, hubBore, 10, hubW, -hubW),
			units.Millimeters(keywayRho))
		require.NoError(t, err)
		sec := hubFillet(keywayRho)
		require.InDelta(t, shoelaceArea(hubRegion(keywayRho)), sec.corner, 1e-7)
		requireKeywayBlend(t, body, filleted, sec, 2, 20, 4, 0)
	})

	t.Run("a cavity crossing the cap", func(t *testing.T) {
		t.Parallel()
		// The cavity straddles the end wall x = 60 inside the corner at
		// (3, √91). Its nearest point to the fillet's axis at (3.5, √78) lies
		// 0.65 mm from it, outside the 0.5 mm ball, so it meets no strip facet;
		// its part at x < 60 lies in the corner the fillet removes, and it
		// crosses the cap the end wall takes.
		body := withCavity(t, keywayShaft(t, keywayX0, 60), 59.8, 3.02, 60.2, 3.12, 9.36, 9.46)
		_, err := decad.Fillet(t.Context(), body, keywayEdges(t, body, keywayR, keywayX0, keywayW),
			units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the cap an end face takes meets another facet of the body at")
	})

	t.Run("refusals", func(t *testing.T) {
		t.Parallel()
		// A pin hole along x into the end wall x = 60 at (2.4, 9) has its rim
		// 0.51 mm from the corner, inside the 0.73 mm the fillet's rows reach
		// on the end wall.
		body := keywayShaft(t, keywayX0, 60)
		pin := xCylinderAt(t, body.Document(), 2.4, 9, 0.3, 55, 65)
		holed, err := decad.Cut(t.Context(), body, pin)
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), holed, keywayEdges(t, holed, keywayR, keywayX0, keywayW),
			units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "another part of an end face comes within the blend's reach of the corner")

		// The cross hole of the subtest above, moved to x = 40 inside the
		// keyway's run: its rim's vertex lies on the setback line in the band.
		body = keywayShaft(t, keywayX0, 60)
		holed, err = decad.Cut(t.Context(), body, keywayCrossHole(t, body, 40))
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), holed, keywayEdges(t, holed, keywayR, keywayX0, keywayW, -keywayW),
			units.Millimeters(keywayRho))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a vertex of the wall lies on the blend's setback line")

		// A fillet r = 0.007 of a keyway from x = 20.37 running off the
		// shaft's end: its tangent point on the shaft lies under minSine of
		// the edge's angle about the axis from the edge.
		body = keywayShaft(t, 20.37, 90)
		_, err = decad.Fillet(t.Context(), body, keywayEdges(t, body, keywayR, 20.37, keywayW, -keywayW),
			units.Millimeters(0.007))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the blend's tangent point does not lie on the wall's face")

		// A groove 20 mm long cut into a plate's top face: the top face wraps
		// the groove's flat ends, so beside the convex edge the plane, not the
		// wall, is reflex at each end, and the route builds no cap there.
		sc := newRetentionScene()
		s, err := sc.w.CreateSketch(sc.w.YZ())
		require.NoError(t, err)
		c := s.CreatePoint(10, 22)
		s.Fix(c)
		s.CreateCircle(c, 4)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		tool, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
		require.NoError(t, err)
		shift, err := r3.Translation(r3.Vec{X: 10})
		require.NoError(t, err)
		tool, err = decad.Transform(t.Context(), tool, shift)
		require.NoError(t, err)
		grooved, err := decad.Cut(t.Context(), blendPlate(t, sc), tool)
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), grooved,
			straightEdge(t, grooved, r3.NewVec(10, 10-math.Sqrt(12), 20), r3.NewVec(1, 0, 0)), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a convex edge ends where its plane is reflex")
	})
}

// keywayCrossHole is a hole of radius 0.3 along z through the shaft at x,
// placed so its rim's vertex at x + 0.3 lies on the setback line of the
// fillet r = 0.5, y = 3.5·10/9.5 on the shaft's top.
func keywayCrossHole(t *testing.T, b *decad.Body, x float64) *decad.Body {
	t.Helper()
	sc := &retentionScene{w: sketch.NewWorld(), doc: b.Document()}
	return zCylinderAt(t, sc, x, (keywayW+keywayRho)*keywayR/(keywayR-keywayRho), 0.3, -12, 24)
}

// hubBore is the radius of the hub's bore and hubW half its keyway's width.
const (
	hubBore = 5.0
	hubW    = 2.0
)

// hubKeyway is a hub along x from 0 to 40 with outer radius 10 and a bore of
// radius 5, less the keyway |y| ≤ 2, z ≤ 7.75 from x = 10 to 30.
func hubKeyway(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, hubBore, 40, 10)
	s.Fix(r.A)
	s.Fix(r.C)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	hub, err := decad.Revolve(t.Context(), sc.doc, s, s.Profiles()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	tool := placed(t, rectBox(t, sc.w, sc.doc, 10, -hubW, 30, hubW, 7.5), 0.25)
	cut, err := decad.Cut(t.Context(), hub, tool)
	require.NoError(t, err)
	return cut
}

// hubFillet is the section of a fillet of radius r of the hub keyway's top
// edge at K = (2, √21) in (y, z), between the wall y = 2 (material at y > 2)
// and the bore's circle (material outside it). The ball lies in the hub,
// touching the wall at ta and the circle at tb, with its centre c at distance
// R + r from the axis. The region is the quadrilateral K, ta, c, tb less the
// ball's sector between ta and tb and less the bore's segment between tb and
// K, which bulges toward c.
func hubFillet(r float64) keywaySection {
	k, c, ta, tb := hubPoints(r)
	phi := math.Acos(((ta[0]-c[0])*(tb[0]-c[0]) + (ta[1]-c[1])*(tb[1]-c[1])) / (r * r))
	theta := math.Acos((k[0]*tb[0] + k[1]*tb[1]) / (hubBore * hubBore))
	quad := shoelaceArea([][2]float64{k, ta, c, tb})
	segment := hubBore * hubBore / 2 * (theta - math.Sin(theta))
	return keywaySection{corner: quad - r*r*phi/2 - segment, wall: ta[1] - k[1], theta: theta, width: r * phi,
		radius: hubBore}
}

// hubPoints are the corner K, the ball's centre c and the tangent points ta
// and tb of hubFillet.
func hubPoints(r float64) (k, c, ta, tb [2]float64) {
	k = [2]float64{hubW, math.Sqrt(hubBore*hubBore - hubW*hubW)}
	c = [2]float64{hubW + r, math.Sqrt((hubBore+r)*(hubBore+r) - (hubW+r)*(hubW+r))}
	ta = [2]float64{hubW, c[1]}
	scale := hubBore / (hubBore + r)
	tb = [2]float64{c[0] * scale, c[1] * scale}
	return k, c, ta, tb
}

// hubRegion samples hubFillet's corner region as a polygon: K, ta, the ball's
// short arc to tb, and the bore's circle back to K.
func hubRegion(r float64) [][2]float64 {
	const n = 4000
	k, c, ta, tb := hubPoints(r)
	arc := func(o [2]float64, rad float64, from, to [2]float64) [][2]float64 {
		a0 := math.Atan2(from[1]-o[1], from[0]-o[0])
		sweep := math.Atan2(to[1]-o[1], to[0]-o[0]) - a0
		if sweep > math.Pi {
			sweep -= 2 * math.Pi
		}
		if sweep < -math.Pi {
			sweep += 2 * math.Pi
		}
		var out [][2]float64
		for i := 1; i < n; i++ {
			a := a0 + sweep*float64(i)/n
			out = append(out, [2]float64{o[0] + rad*math.Cos(a), o[1] + rad*math.Sin(a)})
		}
		return out
	}
	region := [][2]float64{k, ta}
	region = append(region, arc(c, r, ta, tb)...)
	region = append(region, tb)
	return append(region, arc([2]float64{}, hubBore, tb, k)...)
}

// keywayRegion samples the corner region of a fillet (or chamfer) of size s
// as a polygon: K, ta, the blend from ta to tb, and the shaft's circle back to
// K.
func keywayRegion(s float64, fillet bool) [][2]float64 {
	const n = 4000
	k := [2]float64{keywayW, math.Sqrt(keywayR*keywayR - keywayW*keywayW)}
	var ta, tb [2]float64
	region := [][2]float64{k}
	if fillet {
		cy := keywayW + s
		cz := math.Sqrt((keywayR-s)*(keywayR-s) - cy*cy)
		ta = [2]float64{keywayW, cz}
		tb = [2]float64{cy * keywayR / (keywayR - s), cz * keywayR / (keywayR - s)}
		a0, a1 := math.Atan2(ta[1]-cz, ta[0]-cy), math.Atan2(tb[1]-cz, tb[0]-cy)
		for i := 0; i <= n; i++ {
			a := a0 + (a1-a0)*float64(i)/n
			region = append(region, [2]float64{cy + s*math.Cos(a), cz + s*math.Sin(a)})
		}
	} else {
		alpha := math.Asin(keywayW / keywayR)
		ta = [2]float64{keywayW, k[1] - s}
		tb = [2]float64{keywayR * math.Sin(alpha+s/keywayR), keywayR * math.Cos(alpha+s/keywayR)}
		region = append(region, ta, tb)
	}
	b0, b1 := math.Atan2(tb[1], tb[0]), math.Atan2(k[1], k[0])
	for i := 1; i < n; i++ {
		a := b0 + (b1-b0)*float64(i)/n
		region = append(region, [2]float64{keywayR * math.Cos(a), keywayR * math.Sin(a)})
	}
	return region
}

// xCylinderAt is a cylinder of radius r along x at (y, z) = (cy, cz) from x0
// to x1, placed by a translation so a Boolean takes the mesh path.
func xCylinderAt(t *testing.T, doc *decad.Document, cy, cz, r, x0, x1 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.YZ())
	require.NoError(t, err)
	c := s.CreatePoint(cy, cz)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(x1 - x0), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{X: x0 + 0.001})
	require.NoError(t, err)
	out, err := decad.Transform(t.Context(), pin, move)
	require.NoError(t, err)
	return out
}
