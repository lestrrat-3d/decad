package crossbore_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func stationsAt(phis ...float64) []crossbore.Station {
	out := make([]crossbore.Station, len(phis))
	for i, p := range phis {
		out[i] = crossbore.Station{Phi: p}
	}
	return out
}

func TestStationGapRefusesNonIncreasingAngles(t *testing.T) {
	for _, tc := range []struct {
		name string
		phis []float64
		want string
	}{
		{"equal neighbours", []float64{0, 1, 1, 2}, "an outer-ring angle is not increasing"},
		{"backward pair", []float64{1, 0}, "an outer-ring angle is not increasing"},
		{"wrap passes a full turn", []float64{0, 1, 7}, "an outer-ring wrap is not increasing"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := crossbore.StationGap(stationsAt(tc.phis...))
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}

func TestStationGapReadsWrapAndPanelGaps(t *testing.T) {
	twoPiUpper := new(big.Rat).Mul(proofbound.PiUpper, big.NewRat(2, 1))
	// The wrap panel (2*PiUpper - 3) is wider than every neighbour gap (1).
	got, err := crossbore.StationGap(stationsAt(0, 1, 2, 3))
	require.NoError(t, err)
	wrap := new(big.Rat).Sub(twoPiUpper, big.NewRat(3, 1))
	require.Equal(t, proofbound.RatFloatUp(wrap), got)
	// A neighbour gap of 4 is wider than the wrap panel (2*PiUpper - 5).
	got, err = crossbore.StationGap(stationsAt(0, 1, 5))
	require.NoError(t, err)
	require.Equal(t, 4.0, got)
}

func TestStationPlanOrdersEveryMouthStation(t *testing.T) {
	rec := &facetedrecord.CrossCylinderBoreRecord{Radius: 5, Bore: 1, Height: 10, CenterZ: 5}
	stations, err := crossbore.StationPlan(rec, 0.5)
	require.NoError(t, err)
	require.Len(t, stations, 2*(crossbore.HalfStations+511+1))
	counts := map[int]int{}
	for i, s := range stations {
		counts[s.Kind]++
		if i > 0 {
			require.Less(t, stations[i-1].Phi, s.Phi, "station %d", i)
		}
	}
	require.Equal(t, 2*(crossbore.HalfStations+1), counts[crossbore.Positive]+counts[crossbore.Negative])
	require.Equal(t, crossbore.HalfStations+1, counts[crossbore.Positive])
	require.Equal(t, crossbore.HalfStations+1, counts[crossbore.Negative])
	require.Equal(t, 2*511, counts[crossbore.Rest])
	// The positive mouth is at +X and the negative mouth at -X.
	for _, s := range stations {
		switch s.Kind {
		case crossbore.Positive:
			require.Positive(t, s.Upper.X)
		case crossbore.Negative:
			require.Negative(t, s.Upper.X)
		}
	}
	_, err = crossbore.StationPlan(rec, 20)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "a cross-bore mouth turns backward")
	// A zero bore puts every positive-mouth station at one angle: equal
	// neighbours are refused like ones that go backward.
	_, err = crossbore.StationPlan(&facetedrecord.CrossCylinderBoreRecord{Radius: 5, Height: 10, CenterZ: 5}, 0)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "a cross-bore mouth turns backward")
}

func TestFeetMeetTheSourceSeamAtZeroSetback(t *testing.T) {
	rec := &facetedrecord.CrossCylinderBoreRecord{Radius: 5, Bore: 1, Height: 10, CenterZ: 5}
	for _, theta := range []float64{0, 0.7, 1.5707963267948966, 3, 4.5, 6} {
		outer, inner := crossbore.Feet(rec, 0, theta)
		// With no setback both feet are the seam point
		// x = sqrt(R^2-r^2 cos^2 t), y = r cos t, z = c + r sin t.
		require.InDelta(t, outer.X, inner.X, 1e-12)
		require.InDelta(t, outer.Y, inner.Y, 1e-12)
		require.InDelta(t, outer.Z, inner.Z, 1e-12)
		require.InDelta(t, 5*5, outer.X*outer.X+outer.Y*outer.Y, 1e-9)
		require.InDelta(t, 1*1, outer.Y*outer.Y+(outer.Z-5)*(outer.Z-5), 1e-9)
	}
}

// TestFeetSitAtTheSetbackOnTheDevelopedCylinders reads each foot's distance
// from the seam point on its own unrolled cylinder. The outer foot unrolls the
// target cylinder (arc length R*angle, height z); the bore foot unrolls the
// bore cylinder (arc length r*angle about the X axis, distance along X). Both
// distances equal the setback.
func TestFeetSitAtTheSetbackOnTheDevelopedCylinders(t *testing.T) {
	const R, r, c, setback = 5.0, 1.0, 5.0, 0.5
	rec := &facetedrecord.CrossCylinderBoreRecord{Radius: R, Bore: r, Height: 10, CenterZ: c}
	for _, theta := range []float64{0.3, 0.7, 1.2, 2.0, 2.6, 3.4, 4.1, 5.0, 5.9} {
		co, sn := math.Cos(theta), math.Sin(theta)
		x := math.Sqrt(R*R - r*r*co*co)
		outer, inner := crossbore.Feet(rec, setback, theta)

		dAngle := math.Atan2(outer.Y, outer.X) - math.Atan2(r*co, x)
		dz := outer.Z - (c + r*sn)
		require.InDelta(t, setback, math.Hypot(R*dAngle, dz), 1e-9, "outer foot at %g", theta)

		dBoreAngle := math.Atan2(inner.Z-c, inner.Y) - math.Atan2(r*sn, r*co)
		dx := inner.X - x
		require.InDelta(t, setback, math.Hypot(r*dBoreAngle, dx), 1e-9, "bore foot at %g", theta)
	}
}

func TestCapPatchesGroupFanTrianglesBySource(t *testing.T) {
	patches := crossbore.CapPatches([]int{0, 1, 2, -1, 0, 1, 3})
	require.Len(t, patches, 2)
	require.Equal(t, [2]int{0, 1}, patches[0].Axes)
	require.Equal(t, [2]int{0, 1}, patches[1].Axes)
	require.Equal(t, []int{0, 4}, patches[0].Triangles)
	require.Equal(t, []int{1, 5}, patches[1].Triangles)
	require.Empty(t, crossbore.CapPatches(nil)[0].Triangles)
}

func TestMeshBuildersAppendIndexedPrimitives(t *testing.T) {
	m := &crossbore.Mesh{}
	require.Equal(t, 0, m.Point(r3.Vec{X: 1}))
	require.Equal(t, 1, m.Point(r3.Vec{Y: 2}))
	m.Triangle(0, 1, 2, 7)
	m.Quad(3, 4, 5, 6, 9)
	require.Equal(t, []r3.Vec{{X: 1}, {Y: 2}}, m.Verts)
	require.Equal(t, [][3]int{{0, 1, 2}, {3, 4, 5}, {3, 5, 6}}, m.Tris)
	require.Equal(t, []int{7, 9, 9}, m.Src)
}

func TestStationErrorEnclosesTheSourceCircle(t *testing.T) {
	radius := proofarith.FloatRat(5)
	// A point on the circle at angle phi reads a small positive bound; one
	// moved radially by 1e-3 reads a bound of at least that distance.
	on := r3.Vec{X: 5, Y: 0}
	e, ok := crossbore.StationError(on, radius, 0)
	require.True(t, ok)
	require.Less(t, e, 1e-12)
	off, ok := crossbore.StationError(r3.Vec{X: 5.001}, radius, 0)
	require.True(t, ok)
	require.GreaterOrEqual(t, off, 1e-3)
}
