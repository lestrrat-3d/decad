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
