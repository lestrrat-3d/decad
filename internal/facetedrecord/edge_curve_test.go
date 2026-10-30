package facetedrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// arcRadius is the radius of every chain arcChain builds.
const arcRadius = 4

func retained(s surfacegeom.Surface) *topo.Face { return &topo.Face{Surface: s, RetainedTag: true} }

func planeAt(t *testing.T, origin, u, v r3.Vec) surfacegeom.Plane {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return surfacegeom.Plane{Frame: frame}
}

func arcChain(centre r3.Vec, from, to float64, steps int) ([]r3.Vec, []float64) {
	var chain []r3.Vec
	var bounds []float64
	for i := 0; i <= steps; i++ {
		a := from + (to-from)*float64(i)/float64(steps)
		chain = append(chain, centre.Add(r3.NewVec(arcRadius*math.Cos(a), arcRadius*math.Sin(a), 0)))
		bounds = append(bounds, 1e-6)
	}
	return chain, bounds
}

func TestRetainedEdgeCurvePlanePlane(t *testing.T) {
	t.Parallel()
	floor := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
	wall := retained(planeAt(t, r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)))
	chain := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(0, 3, 0), r3.NewVec(0, 7, 0)}
	bounds := []float64{1e-9, 1e-9, 1e-9}

	curve, ok := facetedrecord.RetainedEdgeCurve(floor, wall, chain, bounds, false)
	require.True(t, ok)
	require.IsType(t, surfacegeom.Line3{}, curve)

	// The falsifier rejects a vertex far off the plane pair's line.
	off := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(0, 3, 0.01), r3.NewVec(0, 7, 0)}
	_, ok = facetedrecord.RetainedEdgeCurve(floor, wall, off, bounds, false)
	require.False(t, ok, "a vertex 10⁻² off the line, bound 10⁻⁹")
	// ...and accepts it once that vertex's own bound covers the offset.
	_, ok = facetedrecord.RetainedEdgeCurve(floor, wall, off, []float64{1e-9, 0.02, 1e-9}, false)
	require.True(t, ok)

	_, ok = facetedrecord.RetainedEdgeCurve(floor, floor, chain, bounds, false)
	require.False(t, ok, "one face cannot meet itself along a rim")
	_, ok = facetedrecord.RetainedEdgeCurve(floor, &topo.Face{Surface: wall.Surface}, chain, bounds, false)
	require.False(t, ok, "a face with no retained tag keeps the chord chain")
	parallel := retained(planeAt(t, r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
	_, ok = facetedrecord.RetainedEdgeCurve(floor, parallel, chain, bounds, false)
	require.False(t, ok, "parallel planes meet nowhere")
}

func TestRetainedEdgeCurveChargesPlaneAxialDelta(t *testing.T) {
	t.Parallel()
	wall := retained(planeAt(t, r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)))
	chain := []r3.Vec{r3.NewVec(0, 0, 1e-8), r3.NewVec(0, 3, 1e-8), r3.NewVec(0, 7, 1e-8)}
	bounds := []float64{1e-9, 1e-9, 1e-9}
	floor := func(delta float64) *topo.Face {
		f := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
		f.HasAxialDelta, f.AxialDelta = true, delta
		return f
	}

	_, ok := facetedrecord.RetainedEdgeCurve(floor(0), wall, chain, bounds, false)
	require.False(t, ok, "a rim 1e-8 off an exact floor, bound 1e-9, is falsified")
	curve, ok := facetedrecord.RetainedEdgeCurve(floor(1e-8), wall, chain, bounds, false)
	require.True(t, ok, "a floor stated to lie 1e-8 off its tag charges that into the falsifier")
	require.IsType(t, surfacegeom.Line3{}, curve, "only a line carries the widened falsifier")
	for _, bad := range []float64{-1, math.Inf(1), math.NaN()} {
		_, ok = facetedrecord.RetainedEdgeCurve(floor(bad), wall, chain, bounds, false)
		require.False(t, ok, "a displacement no bound can cover declines the rim: %v", bad)
	}
}

func TestRetainedEdgeCurveDisplacedPlaneDeclinesConic(t *testing.T) {
	t.Parallel()
	displaced := func(f *topo.Face, delta float64) *topo.Face {
		f.HasAxialDelta, f.AxialDelta = true, delta
		return f
	}

	t.Run("a circle rim", func(t *testing.T) {
		t.Parallel()
		floor := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
		bore := retained(surfacegeom.Cylinder{Origin: r3.NewVec(10, 10, 0), Axis: r3.NewVec(0, 0, 1),
			Radius: units.Millimeters(4)})
		ring, bounds := arcChain(r3.NewVec(10, 10, 0), 0, 2*math.Pi, 8)
		ring[len(ring)-1] = ring[0]

		_, ok := facetedrecord.RetainedEdgeCurve(displaced(floor, 1e-8), bore, ring, bounds, true)
		require.False(t, ok, "a circle is published exact: it cannot carry a displaced plane")
		curve, ok := facetedrecord.RetainedEdgeCurve(displaced(floor, 2.9e-16), bore, ring, bounds, true)
		require.True(t, ok, "a unit-conversion sized delta stays inside the arithmetic allowance")
		require.IsType(t, surfacegeom.Circle3{}, curve)
		curve, ok = facetedrecord.RetainedEdgeCurve(displaced(floor, 0), bore, ring, bounds, true)
		require.True(t, ok)
		require.IsType(t, surfacegeom.Circle3{}, curve)
	})

	t.Run("an arc rim", func(t *testing.T) {
		t.Parallel()
		floor := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
		bore := retained(surfacegeom.Cylinder{Origin: r3.NewVec(10, 10, 0), Axis: r3.NewVec(0, 0, 1),
			Radius: units.Millimeters(4)})
		chain, bounds := arcChain(r3.NewVec(10, 10, 0), 0.2, 2.5, 6)
		_, ok := facetedrecord.RetainedEdgeCurve(displaced(floor, 1e-8), bore, chain, bounds, false)
		require.False(t, ok)
	})

	t.Run("an ellipse rim", func(t *testing.T) {
		t.Parallel()
		floor, bore, _, point := obliqueScene(t)
		ring, bounds := ellipseChain(point, 0, 2*math.Pi, 12)
		ring[len(ring)-1] = ring[0]
		_, ok := facetedrecord.RetainedEdgeCurve(displaced(floor, 1e-8), bore, ring, bounds, true)
		require.False(t, ok, "an ellipse is published exact: it cannot carry a displaced plane")
		curve, ok := facetedrecord.RetainedEdgeCurve(displaced(floor, 2.9e-16), bore, ring, bounds, true)
		require.True(t, ok)
		require.IsType(t, surfacegeom.Ellipse3{}, curve)
	})

	t.Run("a sphere circle", func(t *testing.T) {
		t.Parallel()
		floor := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
		ball := retained(surfacegeom.Sphere{Center: r3.NewVec(0, 0, 3), Radius: units.Millimeters(5)})
		ring, bounds := arcChain(r3.Vec{}, 0, 2*math.Pi, 8)
		ring[len(ring)-1] = ring[0]
		_, ok := facetedrecord.RetainedEdgeCurve(displaced(floor, 1e-8), ball, ring, bounds, true)
		require.False(t, ok)
	})

	t.Run("a generator line of a tangent cylinder", func(t *testing.T) {
		t.Parallel()
		floor := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
		blend := retained(surfacegeom.Cylinder{Origin: r3.NewVec(3, 0, 3), Axis: r3.NewVec(0, 1, 0),
			Radius: units.Millimeters(3)})
		chain := []r3.Vec{r3.NewVec(3, 0, 1e-8), r3.NewVec(3, 5, 1e-8), r3.NewVec(3, 9, 1e-8)}
		bounds := []float64{1e-9, 1e-9, 1e-9}
		_, ok := facetedrecord.RetainedEdgeCurve(floor, blend, chain, bounds, false)
		require.False(t, ok)
		curve, ok := facetedrecord.RetainedEdgeCurve(displaced(floor, 1e-8), blend, chain, bounds, false)
		require.True(t, ok)
		require.IsType(t, surfacegeom.Line3{}, curve)
	})
}

// TestRetainedEdgeCurveDepartedTagKeepsChords pairs faces whose tags state a
// departure (§10.1 E12): every rim but a plane-plane line keeps its chords.
func TestRetainedEdgeCurveDepartedTagKeepsChords(t *testing.T) {
	t.Parallel()
	departed := func(f *topo.Face) *topo.Face {
		f.TagDeparture = 0x1p-50
		return f
	}
	floor := func() *topo.Face {
		return retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
	}
	bore := func() *topo.Face {
		return retained(surfacegeom.Cylinder{Origin: r3.NewVec(10, 10, 0), Axis: r3.NewVec(0, 0, 1),
			Radius: units.Millimeters(4)})
	}
	ring, ringBounds := arcChain(r3.NewVec(10, 10, 0), 0, 2*math.Pi, 8)
	ring[len(ring)-1] = ring[0]

	curve, ok := facetedrecord.RetainedEdgeCurve(floor(), bore(), ring, ringBounds, true)
	require.True(t, ok)
	require.IsType(t, surfacegeom.Circle3{}, curve, "two exact tags publish the bore circle")
	_, ok = facetedrecord.RetainedEdgeCurve(floor(), departed(bore()), ring, ringBounds, true)
	require.False(t, ok, "a departed cylinder keeps the bore rim's chords")
	_, ok = facetedrecord.RetainedEdgeCurve(departed(floor()), bore(), ring, ringBounds, true)
	require.False(t, ok, "a departed plane keeps the bore rim's chords")
	_, ok = facetedrecord.RetainedEdgeCurve(floor(), &topo.Face{Surface: bore().Surface, RetainedTag: true,
		TagDeparture: math.NaN()}, ring, ringBounds, true)
	require.False(t, ok, "a departure that is not a number keeps the chords")

	arc, arcBounds := arcChain(r3.NewVec(10, 10, 0), 0.2, 2.5, 6)
	_, ok = facetedrecord.RetainedEdgeCurve(floor(), departed(bore()), arc, arcBounds, false)
	require.False(t, ok, "a departed cylinder keeps an arc rim's chords")

	tilted, cyl, _, point := obliqueScene(t)
	ellipse, ellipseBounds := ellipseChain(point, 0, 2*math.Pi, 12)
	ellipse[len(ellipse)-1] = ellipse[0]
	_, ok = facetedrecord.RetainedEdgeCurve(tilted, departed(cyl), ellipse, ellipseBounds, true)
	require.False(t, ok, "a departed cylinder keeps an ellipse rim's chords")

	ball := departed(retained(surfacegeom.Sphere{Center: r3.NewVec(0, 0, 3), Radius: units.Millimeters(5)}))
	sphereRing, sphereBounds := arcChain(r3.Vec{}, 0, 2*math.Pi, 8)
	sphereRing[len(sphereRing)-1] = sphereRing[0]
	_, ok = facetedrecord.RetainedEdgeCurve(floor(), ball, sphereRing, sphereBounds, true)
	require.False(t, ok, "a departed sphere keeps its circle's chords")

	blend := departed(retained(surfacegeom.Cylinder{Origin: r3.NewVec(3, 0, 3), Axis: r3.NewVec(0, 1, 0),
		Radius: units.Millimeters(3)}))
	line := []r3.Vec{r3.NewVec(3, 0, 0), r3.NewVec(3, 5, 0), r3.NewVec(3, 9, 0)}
	lineBounds := []float64{1e-9, 1e-9, 1e-9}
	_, ok = facetedrecord.RetainedEdgeCurve(floor(), blend, line, lineBounds, false)
	require.False(t, ok, "a departed cylinder keeps a generator rim's chords")

	wall := departed(retained(planeAt(t, r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))))
	curve, ok = facetedrecord.RetainedEdgeCurve(departed(floor()), wall,
		[]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(0, 3, 0), r3.NewVec(0, 7, 0)}, lineBounds, false)
	require.True(t, ok, "two departed planes still meet in a line")
	require.IsType(t, surfacegeom.Line3{}, curve)
}

func TestRetainedEdgeCurvePlaneCylinder(t *testing.T) {
	t.Parallel()
	floor := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
	// A blend of radius 3 whose axis runs along y at x=3, z=3: it touches the
	// floor z=0 along the generator x=3, tangentially.
	blend := retained(surfacegeom.Cylinder{Origin: r3.NewVec(3, 0, 3), Axis: r3.NewVec(0, 1, 0), Radius: units.Millimeters(3)})
	chain := []r3.Vec{r3.NewVec(3, 0, 0), r3.NewVec(3, 5, 0), r3.NewVec(3, 9, 0)}
	bounds := []float64{1e-9, 1e-9, 1e-9}

	curve, ok := facetedrecord.RetainedEdgeCurve(floor, blend, chain, bounds, false)
	require.True(t, ok)
	require.IsType(t, surfacegeom.Line3{}, curve, "a tangent contact is one generator line")
	curve, ok = facetedrecord.RetainedEdgeCurve(blend, floor, chain, bounds, false)
	require.True(t, ok, "the order of the two faces does not matter")
	require.IsType(t, surfacegeom.Line3{}, curve)

	// A bore along z through the floor meets it in a circle of the bore's radius.
	bore := retained(surfacegeom.Cylinder{Origin: r3.NewVec(10, 10, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(4)})
	ring, ringBounds := arcChain(r3.NewVec(10, 10, 0), 0, 2*math.Pi, 8)
	ring[len(ring)-1] = ring[0]
	curve, ok = facetedrecord.RetainedEdgeCurve(floor, bore, ring, ringBounds, true)
	require.True(t, ok)
	circle, isCircle := curve.(surfacegeom.Circle3)
	require.True(t, isCircle)
	require.Equal(t, r3.NewVec(10, 10, 0), circle.Center)
	require.InDelta(t, 4, circle.Radius.Base(), 1e-12)

	// A vertex 0.5 mm off the bore's circle, bound 10⁻⁶, falsifies the pairing.
	bad := append([]r3.Vec(nil), ring...)
	bad[2] = bad[2].Add(r3.NewVec(0.5, 0, 0))
	_, ok = facetedrecord.RetainedEdgeCurve(floor, bore, bad, ringBounds, true)
	require.False(t, ok)
}

func TestRetainedEdgeCurveArcSense(t *testing.T) {
	t.Parallel()
	floor := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
	bore := retained(surfacegeom.Cylinder{Origin: r3.NewVec(10, 10, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(4)})
	centre := r3.NewVec(10, 10, 0)

	ccw, bounds := arcChain(centre, 0.2, 2.5, 6)
	curve, ok := facetedrecord.RetainedEdgeCurve(floor, bore, ccw, bounds, false)
	require.True(t, ok)
	arc, isArc := curve.(surfacegeom.Arc3)
	require.True(t, isArc)
	require.Equal(t, 1.0, arc.Axis.Z, "a counter-clockwise run sweeps about +z")

	cw, bounds := arcChain(centre, 2.5, 0.2, 6)
	curve, ok = facetedrecord.RetainedEdgeCurve(floor, bore, cw, bounds, false)
	require.True(t, ok)
	arc = curve.(surfacegeom.Arc3)
	require.Equal(t, -1.0, arc.Axis.Z, "a clockwise run sweeps about −z")

	two, twoBounds := arcChain(centre, 0.2, 2.5, 1)
	_, ok = facetedrecord.RetainedEdgeCurve(floor, bore, two, twoBounds, false)
	require.False(t, ok, "two vertices do not say which way round a rim runs")

	wobble := append([]r3.Vec(nil), ccw...)
	wobble[3], wobble[2] = wobble[2], wobble[3]
	_, ok = facetedrecord.RetainedEdgeCurve(floor, bore, wobble, bounds, false)
	require.False(t, ok, "a chain that doubles back has no single sense")
}

// obliqueScene is the floor z=0 and a Ø8 bore whose axis leans 30° from z
// toward +x and crosses the floor at the origin.
func obliqueScene(t *testing.T) (floor, bore *topo.Face, tilt float64, ellipsePoint func(phi float64) r3.Vec) {
	t.Helper()
	tilt = 30 * math.Pi / 180
	floor = retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
	axis := r3.NewVec(math.Sin(tilt), 0, math.Cos(tilt))
	bore = retained(surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: axis, Radius: units.Millimeters(4)})
	ellipsePoint = func(phi float64) r3.Vec {
		return r3.NewVec(4/math.Cos(tilt)*math.Cos(phi), 4*math.Sin(phi), 0)
	}
	return floor, bore, tilt, ellipsePoint
}

func ellipseChain(point func(float64) r3.Vec, from, to float64, steps int) ([]r3.Vec, []float64) {
	var chain []r3.Vec
	var bounds []float64
	for i := 0; i <= steps; i++ {
		chain = append(chain, point(from+(to-from)*float64(i)/float64(steps)))
		bounds = append(bounds, 1e-6)
	}
	return chain, bounds
}

func TestRetainedEdgeCurveObliqueEllipse(t *testing.T) {
	t.Parallel()
	floor, bore, tilt, point := obliqueScene(t)

	t.Run("a closed rim is one whole ellipse", func(t *testing.T) {
		t.Parallel()
		ring, bounds := ellipseChain(point, 0, 2*math.Pi, 12)
		ring[len(ring)-1] = ring[0]
		curve, ok := facetedrecord.RetainedEdgeCurve(floor, bore, ring, bounds, true)
		require.True(t, ok)
		e, isEllipse := curve.(surfacegeom.Ellipse3)
		require.True(t, isEllipse)
		require.InDelta(t, 4, e.SemiMinor.Base(), 1e-12)
		require.InDelta(t, 4/math.Cos(tilt), e.SemiMajor.Base(), 1e-12)
		require.InDelta(t, 0, e.Center.Len(), 1e-12)
		require.InDelta(t, 1, e.Major.X, 1e-12, "the major axis lies along the tilt")
		require.Equal(t, 1.0, e.Axis.Z)
		// The same curve from the other face order.
		_, ok = facetedrecord.RetainedEdgeCurve(bore, floor, ring, bounds, true)
		require.True(t, ok)
	})

	t.Run("an open rim runs the way its vertices do", func(t *testing.T) {
		t.Parallel()
		ccw, bounds := ellipseChain(point, 0.3, 2.6, 6)
		curve, ok := facetedrecord.RetainedEdgeCurve(floor, bore, ccw, bounds, false)
		require.True(t, ok)
		require.Equal(t, 1.0, curve.(surfacegeom.Ellipse3).Axis.Z)
		cw, bounds := ellipseChain(point, 2.6, 0.3, 6)
		curve, ok = facetedrecord.RetainedEdgeCurve(floor, bore, cw, bounds, false)
		require.True(t, ok)
		require.Equal(t, -1.0, curve.(surfacegeom.Ellipse3).Axis.Z)
	})

	t.Run("a vertex off the plane or off the cylinder declines", func(t *testing.T) {
		t.Parallel()
		ring, bounds := ellipseChain(point, 0, 2*math.Pi, 12)
		ring[len(ring)-1] = ring[0]
		offPlane := append([]r3.Vec(nil), ring...)
		offPlane[3] = offPlane[3].Add(r3.NewVec(0, 0, 0.5))
		_, ok := facetedrecord.RetainedEdgeCurve(floor, bore, offPlane, bounds, true)
		require.False(t, ok, "0.5 mm off the plane, bound 10⁻⁶")
		offCylinder := append([]r3.Vec(nil), ring...)
		offCylinder[3] = offCylinder[3].Add(r3.NewVec(0.5, 0, 0))
		_, ok = facetedrecord.RetainedEdgeCurve(floor, bore, offCylinder, bounds, true)
		require.False(t, ok, "0.5 mm off the cylinder, bound 10⁻⁶")
	})

	t.Run("an axis nearly in the plane declines", func(t *testing.T) {
		t.Parallel()
		k := 5e-4
		flat := retained(surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: r3.NewVec(math.Sqrt(1-k*k), 0, k),
			Radius: units.Millimeters(4)})
		ring, bounds := ellipseChain(point, 0, 2*math.Pi, 12)
		ring[len(ring)-1] = ring[0]
		_, ok := facetedrecord.RetainedEdgeCurve(floor, flat, ring, bounds, true)
		require.False(t, ok)
	})

	t.Run("a chain that doubles back declines", func(t *testing.T) {
		t.Parallel()
		ccw, bounds := ellipseChain(point, 0.3, 2.6, 6)
		ccw[3], ccw[2] = ccw[2], ccw[3]
		_, ok := facetedrecord.RetainedEdgeCurve(floor, bore, ccw, bounds, false)
		require.False(t, ok)
	})
}

// TestRetainedEdgeCurveCylinderTorus covers the junction of two fillet blends
// (agent-docs/retained-modify-reach-design.md §5.4, Claim J). A fillet of
// radius 1 along the front edge y=0 of a plate top z=20 is the cylinder about
// the line y=1, z=19. The fillet of the r=3 corner about (3,3) is the torus of
// tube radius 1 about the corner axis at depth 1. They meet in the quarter
// circle about (3,1,19) in the plane x=3.
func TestRetainedEdgeCurveCylinderTorus(t *testing.T) {
	t.Parallel()
	cylinder := func(radius float64) *topo.Face {
		return retained(surfacegeom.Cylinder{Origin: r3.NewVec(3, 1, 19), Axis: r3.NewVec(1, 0, 0),
			Radius: units.Millimeters(radius)})
	}
	torus := func(centre r3.Vec, major float64) *topo.Face {
		return retained(surfacegeom.Torus{Center: centre, Axis: r3.NewVec(0, 0, 1),
			Major: units.Millimeters(major), Minor: units.Millimeters(1)})
	}
	var chain []r3.Vec
	var bounds []float64
	for i := range 7 {
		a := float64(i) / 6 * math.Pi / 2
		chain = append(chain, r3.NewVec(3, 1-math.Sin(a), 19+math.Cos(a)))
		bounds = append(bounds, 1e-9)
	}
	blend, ring := cylinder(1), torus(r3.NewVec(3, 3, 19), 2)

	curve, ok := facetedrecord.RetainedEdgeCurve(blend, ring, chain, bounds, false)
	require.True(t, ok)
	arc, isArc := curve.(surfacegeom.Arc3)
	require.True(t, isArc)
	require.InDelta(t, 1, arc.Radius.Base(), 1e-12)
	require.InDelta(t, 0, arc.Center.Sub(r3.NewVec(3, 1, 19)).Len(), 1e-12)
	require.Equal(t, r3.NewVec(1, 0, 0), arc.Axis, "the run from (3,1,20) to (3,0,19) is counter-clockwise about +x")
	curve, ok = facetedrecord.RetainedEdgeCurve(ring, blend, chain, bounds, false)
	require.True(t, ok, "the order of the two faces does not matter")
	require.IsType(t, surfacegeom.Arc3{}, curve)

	reversed := make([]r3.Vec, len(chain))
	for i, p := range chain {
		reversed[len(chain)-1-i] = p
	}
	curve, ok = facetedrecord.RetainedEdgeCurve(blend, ring, reversed, bounds, false)
	require.True(t, ok)
	require.Equal(t, r3.NewVec(-1, 0, 0), curve.(surfacegeom.Arc3).Axis, "the reversed run sweeps about -x")

	t.Run("a tube centre not at the major radius declines", func(t *testing.T) {
		t.Parallel()
		_, ok := facetedrecord.RetainedEdgeCurve(blend, torus(r3.NewVec(3, 3, 19), 2.5), chain, bounds, false)
		require.False(t, ok)
	})
	t.Run("a torus centre off the equatorial plane declines", func(t *testing.T) {
		t.Parallel()
		// The centre is still Major from the tube centre, but out of the plane.
		_, ok := facetedrecord.RetainedEdgeCurve(blend, torus(r3.NewVec(3, 1+math.Sqrt(3), 18), 2), chain, bounds,
			false)
		require.False(t, ok)
	})
	t.Run("unequal radii decline", func(t *testing.T) {
		t.Parallel()
		// A chain on the cylinder's own circle of radius 1.5 passes the falsifier,
		// but the torus's tube is 1.
		var wide []r3.Vec
		for i := range 7 {
			a := float64(i) / 6 * math.Pi / 2
			wide = append(wide, r3.NewVec(3, 1-1.5*math.Sin(a), 19+1.5*math.Cos(a)))
		}
		_, ok := facetedrecord.RetainedEdgeCurve(cylinder(1.5), ring, wide, bounds, false)
		require.False(t, ok)
	})
	t.Run("a cylinder along the torus axis declines", func(t *testing.T) {
		t.Parallel()
		// Every other identity holds, and a flat circle about the same centre is
		// the cylinder's own section, but the two surfaces do not meet in it.
		along := retained(surfacegeom.Cylinder{Origin: r3.NewVec(3, 1, 19), Axis: r3.NewVec(0, 0, 1),
			Radius: units.Millimeters(1)})
		var flat []r3.Vec
		for i := range 7 {
			a := float64(i) / 6 * math.Pi / 2
			flat = append(flat, r3.NewVec(3+math.Cos(a), 1+math.Sin(a), 19))
		}
		_, ok := facetedrecord.RetainedEdgeCurve(along, ring, flat, bounds, false)
		require.False(t, ok)
	})
	t.Run("a vertex off the circle falsifies the pairing", func(t *testing.T) {
		t.Parallel()
		bad := append([]r3.Vec(nil), chain...)
		bad[3] = bad[3].Add(r3.NewVec(0, 0.5, 0))
		_, ok := facetedrecord.RetainedEdgeCurve(blend, ring, bad, bounds, false)
		require.False(t, ok)
	})
	t.Run("a chain with no interior vertex keeps its chords", func(t *testing.T) {
		t.Parallel()
		_, ok := facetedrecord.RetainedEdgeCurve(blend, ring, []r3.Vec{chain[0], chain[6]}, bounds[:2], false)
		require.False(t, ok)
	})
}

func TestRetainedEdgeCurveParallelCylinders(t *testing.T) {
	t.Parallel()
	cylinder := func(x float64, axis r3.Vec) *topo.Face {
		return retained(surfacegeom.Cylinder{Origin: r3.NewVec(x, 0, 0), Axis: axis, Radius: units.Millimeters(4)})
	}
	left, right := cylinder(0, r3.NewVec(0, 0, 1)), cylinder(7, r3.NewVec(0, 0, 1))
	h := math.Sqrt(16 - 3.5*3.5)
	generator := func(y float64) []r3.Vec {
		return []r3.Vec{r3.NewVec(3.5, y, 0), r3.NewVec(3.5, y, 6), r3.NewVec(3.5, y, 12)}
	}
	bounds := []float64{1e-9, 1e-9, 1e-9}

	for _, y := range []float64{h, -h} {
		curve, ok := facetedrecord.RetainedEdgeCurve(left, right, generator(y), bounds, false)
		require.True(t, ok, "either generator of the pair passes")
		require.IsType(t, surfacegeom.Line3{}, curve)
	}
	_, ok := facetedrecord.RetainedEdgeCurve(right, left, generator(h), bounds, false)
	require.True(t, ok, "the order of the two faces does not matter")

	// The falsifier rejects a chain off both generators, and accepts it once its
	// own bound covers the offset.
	off := []r3.Vec{r3.NewVec(3.5, h, 0), r3.NewVec(3.5, h+0.01, 6), r3.NewVec(3.5, h, 12)}
	_, ok = facetedrecord.RetainedEdgeCurve(left, right, off, bounds, false)
	require.False(t, ok, "a vertex 10⁻² off the generator, bound 10⁻⁹")
	_, ok = facetedrecord.RetainedEdgeCurve(left, right, off, []float64{1e-9, 0.02, 1e-9}, false)
	require.True(t, ok)

	// Axes that cross, circles that do not meet, and circles that touch decline.
	_, ok = facetedrecord.RetainedEdgeCurve(left, cylinder(7, r3.NewVec(1, 0, 0)), generator(h), bounds, false)
	require.False(t, ok, "crossing axes are not a generator")
	_, ok = facetedrecord.RetainedEdgeCurve(left, cylinder(9, r3.NewVec(0, 0, 1)), generator(0), bounds, false)
	require.False(t, ok, "the cross-sections' circles miss")
	_, ok = facetedrecord.RetainedEdgeCurve(left, cylinder(8, r3.NewVec(0, 0, 1)), generator(0), bounds, false)
	require.False(t, ok, "tangent circles are not an edge")
	_, ok = facetedrecord.RetainedEdgeCurve(left, cylinder(0, r3.NewVec(0, 0, 1)), generator(0), bounds, false)
	require.False(t, ok, "one axis cannot carry two cylinders' meeting")
}
