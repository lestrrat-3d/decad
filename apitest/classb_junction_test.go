package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestClassBCrossingVertexFarFromOrigin cuts a 40×20×20 box standing 100 m
// from the origin with a radius-3 drill along y whose centre sits 2.9 below
// the top face, so the hole breaks out of the top. The class-B crossing reach
// pins each of the drill's two crossings with the top face to one vertex, and
// the cylinder's wall, the hole's mouth and both notched y faces meet there.
// The scene that cut the trace placed that vertex through a recorded cut
// parameter, which far from the origin is off the crossing by the
// coordinates' own rounding: 5e-12 mm here, over a hundred times the cut
// allowance its trace length implies. The volume
// is the box less the disc below z = 20 over the 20 mm the drill crosses:
// 16000 − 20·(9π − (9·atan(s/d) − d·s)), d = 20 − 17.1, s = √(9 − d²).
//
// Shown-to-fail: without the keyed vertex's own crossing offset
// (classbgeom.CrossingOffsetUpper in cbBuild.canonicalPoint) the volume reads
// 4.5e-10 mm³ off the exact value under a 2.6e-11 bound.
func TestClassBCrossingVertexFarFromOrigin(t *testing.T) {
	t.Parallel()
	const x0, cz = 1e5, 17.1
	doc := decad.New()

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, 0, x0+40, 20)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)

	tw := sketch.NewWorld()
	plane, err := tw.CreateOffsetPlane(tw.XZ(), -10)
	require.NoError(t, err)
	ts, err := tw.CreateSketch(plane)
	require.NoError(t, err)
	c := ts.CreatePoint(x0+20, cz)
	ts.Fix(c)
	ts.CreateCircle(c, 3)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	drill, err := doc.Extrude(ts, ts.Profiles()[0], decad.Symmetric{D: units.Millimeters(11)})
	require.NoError(t, err)

	got, err := decad.Cut(t.Context(), box, drill)
	require.NoError(t, err)

	d := new(big.Float).SetPrec(junctionPrec).Sub(jf(20), jf(cz))
	root := new(big.Float).SetPrec(junctionPrec).Mul(d, d)
	root.Sub(jf(9), root).Sqrt(root)
	above := junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(root, d))
	above.Mul(above, jf(9))
	above.Sub(above, new(big.Float).SetPrec(junctionPrec).Mul(d, root))
	below := new(big.Float).SetPrec(junctionPrec).Mul(junctionPi(), jf(9))
	below.Sub(below, above).Mul(below, jf(20))
	truth := jf(16000)
	truth.Sub(truth, below)

	volume, err := got.Volume()
	require.NoError(t, err)
	value, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	bound, err := volume.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, volume.Exactness)
	require.Less(t, bound, 1e-6, `the analytic crossing reach, not the mesh path, measures the cut`)
	requireJunctionEncloses(t, `cross-drilled volume`, value, bound, truth)
}
