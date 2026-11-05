package jointinfer_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/jointinfer"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

var params = jointinfer.Params{MaxFit: 1, FitTol: 0.05, AxisTol: 0.5 * math.Pi / 180}

func cyl(x, y, radius, lo, hi float64, convex bool) jointinfer.Cylinder {
	return jointinfer.Cylinder{Origin: r3.Vec{X: x, Y: y}, Axis: r3.Vec{Z: 1}, Radius: radius, Convex: convex, Lo: lo, Hi: hi}
}

func TestMatchBore(t *testing.T) {
	pin := []jointinfer.Cylinder{cyl(10, 10, 3, -2, 15, true)}
	bore := []jointinfer.Cylinder{cyl(10, 10, 3.2, 0, 10, false)}
	ms := jointinfer.MatchBore(pin, bore, params)
	require.Len(t, ms, 1)
	require.InDelta(t, 0.2, ms[0].Clearance, 1e-12)
	require.InDelta(t, 10.0, ms[0].Length, 1e-12)
	require.InDelta(t, 5.0, ms[0].Axis.Point.Z, 1e-12)
	require.InDelta(t, 10.0, ms[0].Axis.Point.X, 1e-12)

	// A pin 0.2 mm off the bore axis, a 2 mm clearance, a bore that does not
	// overlap the pin, a tilted pin and a concave "pin" do not match.
	for name, c := range map[string]struct {
		pin, bore jointinfer.Cylinder
	}{
		"off axis":   {cyl(10.2, 10, 3, -2, 15, true), cyl(10, 10, 3.2, 0, 10, false)},
		"loose":      {cyl(10, 10, 3, -2, 15, true), cyl(10, 10, 5, 0, 10, false)},
		"apart":      {cyl(10, 10, 3, 20, 30, true), cyl(10, 10, 3.2, 0, 10, false)},
		"touching":   {cyl(10, 10, 3, 10, 30, true), cyl(10, 10, 3.2, 0, 10, false)},
		"both bores": {cyl(10, 10, 3, -2, 15, false), cyl(10, 10, 3.2, 0, 10, false)},
	} {
		require.Empty(t, jointinfer.MatchBore([]jointinfer.Cylinder{c.pin}, []jointinfer.Cylinder{c.bore}, params), name)
	}
	tilted := cyl(10, 10, 3, -2, 15, true)
	tilted.Axis = r3.Vec{X: math.Sin(0.02), Z: math.Cos(0.02)}
	require.Empty(t, jointinfer.MatchBore([]jointinfer.Cylinder{tilted}, bore, params))

	// A press fit reads a negative clearance.
	ms = jointinfer.MatchBore([]jointinfer.Cylinder{cyl(10, 10, 3.05, -2, 15, true)},
		[]jointinfer.Cylinder{cyl(10, 10, 3, 0, 10, false)}, params)
	require.Len(t, ms, 1)
	require.InDelta(t, -0.05, ms[0].Clearance, 1e-12)
}

func TestMatchBoreJoinsHalfFaces(t *testing.T) {
	// A cylinder split in two faces is one match covering both.
	pins := []jointinfer.Cylinder{cyl(0, 0, 3, 0, 10, true), cyl(0, 0, 3, 0, 10, true)}
	bores := []jointinfer.Cylinder{cyl(0, 0, 3.2, 0, 10, false), cyl(0, 0, 3.2, 0, 10, false)}
	ms := jointinfer.MatchBore(pins, bores, params)
	require.Len(t, ms, 1)
	require.Equal(t, []int{0, 1}, ms[0].Pin)
	require.Equal(t, []int{0, 1}, ms[0].Bore)
	require.InDelta(t, 10.0, ms[0].Length, 1e-12)
}

func wall(nx float64, offset float64) jointinfer.Plane {
	return jointinfer.Plane{Normal: r3.Vec{X: nx}, Offset: offset,
		Min: r3.Vec{X: offset * nx, Y: -10, Z: 0}, Max: r3.Vec{X: offset * nx, Y: 10, Z: 10}}
}

func TestMatchSlot(t *testing.T) {
	pin := []jointinfer.Cylinder{cyl(0, 0, 2, 0, 10, true)}
	// Walls at x = -2.2 and x = +2.2 face the pin: 4.4 - 4 = 0.4.
	walls := []jointinfer.Plane{wall(-1, 2.2), wall(1, 2.2)}
	walls[0].Offset, walls[0].Normal = -2.2, r3.Vec{X: 1}
	walls[1].Offset, walls[1].Normal = -2.2, r3.Vec{X: -1}
	ms := jointinfer.MatchSlot(pin, walls, params)
	require.Len(t, ms, 1)
	require.InDelta(t, 0.4, ms[0].Clearance, 1e-12)
	require.InDelta(t, 10.0, ms[0].Length, 1e-12)

	// Walls facing away from the pin, or tilted off the pin's axis, do not match.
	away := []jointinfer.Plane{{Normal: r3.Vec{X: -1}, Offset: 2.2, Min: walls[0].Min, Max: walls[0].Max},
		{Normal: r3.Vec{X: 1}, Offset: 2.2, Min: walls[1].Min, Max: walls[1].Max}}
	require.Empty(t, jointinfer.MatchSlot(pin, away, params))
	tilted := []jointinfer.Plane{walls[0], walls[1]}
	tilted[1].Normal = r3.Vec{X: -math.Cos(0.1), Z: math.Sin(0.1)}
	require.Empty(t, jointinfer.MatchSlot(pin, tilted, params))
}

func TestMatchFace(t *testing.T) {
	// A's bottom face (normal -Z at z = 5) rests on B's top face (normal +Z).
	a := jointinfer.Plane{Normal: r3.Vec{Z: -1}, Offset: -5, Min: r3.Vec{X: 0, Y: 0, Z: 5}, Max: r3.Vec{X: 20, Y: 20, Z: 5}}
	b := jointinfer.Plane{Normal: r3.Vec{Z: 1}, Offset: 5, Min: r3.Vec{X: 10, Y: 0, Z: 5}, Max: r3.Vec{X: 30, Y: 20, Z: 5}}
	ms := jointinfer.MatchFace([]jointinfer.Plane{a}, []jointinfer.Plane{b}, params)
	require.Len(t, ms, 1)
	require.InDelta(t, 200.0, ms[0].Area, 1e-9)
	require.InDelta(t, 0.0, ms[0].Gap, 1e-12)
	require.InDelta(t, 15.0, ms[0].Centre.X, 1e-9)
	require.InDelta(t, 5.0, ms[0].Centre.Z, 1e-9)
	require.Equal(t, r3.Vec{Z: -1}, ms[0].Normal)

	// Faces that only meet on an edge, or lie 3 mm apart, do not match.
	edge := b
	edge.Min, edge.Max = r3.Vec{X: 20, Y: 0, Z: 5}, r3.Vec{X: 40, Y: 20, Z: 5}
	require.Empty(t, jointinfer.MatchFace([]jointinfer.Plane{a}, []jointinfer.Plane{edge}, params))
	far := b
	far.Offset, far.Min.Z, far.Max.Z = 2, 2, 2
	require.Empty(t, jointinfer.MatchFace([]jointinfer.Plane{a}, []jointinfer.Plane{far}, params))
	same := b
	same.Normal = r3.Vec{Z: -1}
	same.Offset = -5
	require.Empty(t, jointinfer.MatchFace([]jointinfer.Plane{a}, []jointinfer.Plane{same}, params))
}
