package twistsweep

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkStationedTwistCells builds the stationed body of the tube-wall
// cutter: a 15.4 by 4.75 rectangle turned 87 degrees over 6 mm, placed off
// the axis. "refined" is the 233-cell build RefinedMesh asks for, "base" the
// stationed build's own cell count.
func BenchmarkStationedTwistCells(b *testing.B) {
	corners := [][2]float64{{-7.7, -2.375}, {7.7, -2.375}, {7.7, 2.375}, {-7.7, 2.375}}
	segs := make([]sectionrecord.CurveSegment, len(corners))
	for i, p := range corners {
		q := corners[(i+1)%len(corners)]
		segs[i] = sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: p[0], V: p[1]},
			End:   sectionrecord.Point2{U: q[0], V: q[1]}, TEnd: 1,
		}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
	poly, err := TwistPolygonOf(profile)
	require.NoError(b, err)
	angle := units.Degrees(87)
	theta, err := angle.In(units.Radian)
	require.NoError(b, err)
	thetaIv, ok := revolveangle.FromValue(angle).Enclosure()
	require.True(b, ok)
	absTheta := proofbound.IntervalAbsUpper(thetaIv)
	upper := proofbound.RatFloatUp(absTheta)
	base, _, err := StationedTwistCellCounts(poly, upper)
	require.NoError(b, err)
	refined, err := refinedCellCount(poly, upper)
	require.NoError(b, err)
	xform, err := r3.Translation(r3.NewVec(15.5, 0, 9))
	require.NoError(b, err)
	recordArea, _ := poly.Area.Float64()
	path := &sweepinput.Path{End: r3.NewVec(0, 0, 6)}
	plane := planeRecord{U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	env := topo.Env{Alloc: topotest.Alloc{}}
	for _, bench := range []struct {
		name  string
		cells int
	}{{"refined", refined}, {"base", base}} {
		b.Run(bench.name, func(b *testing.B) {
			for b.Loop() {
				_, err := evalStationedCells(b.Context(), env, profile, plane, path, angle, recordArea, xform,
					poly, theta, thetaIv, absTheta, bench.cells, false)
				require.NoError(b, err)
			}
		})
	}
}
