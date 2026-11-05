package facepair_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// holedPlane is the plane z = 0 trimmed to the square [-10, 10]² with a
// circular hole of radius hole about the origin.
func holedPlane(t *testing.T, hole float64) *clearance.CFace {
	t.Helper()
	var elems []survey2d.SurveyElem
	corners := [][2]float64{{-10, -10}, {10, -10}, {10, 10}, {-10, 10}}
	for i := range corners {
		a, b := corners[i], corners[(i+1)%len(corners)]
		e, ok := survey2d.LineElem(a[0], a[1], b[0], b[1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	e, ok := survey2d.ArcElem(0, 0, hole, 2*math.Pi, 0, true)
	require.True(t, ok)
	elems = append(elems, e)
	f := &clearance.CFace{
		Kind:   clearance.CkPlane,
		O:      r3.Vec{},
		U:      r3.NewVec(1, 0, 0),
		V:      r3.NewVec(0, 1, 0),
		N:      r3.NewVec(0, 0, 1),
		Region: clearance.NewRegion2(elems),
	}
	f.Box = clearance.CapBox(f)
	f.Wit = []r3.Vec{r3.NewVec(5, 5, 0)}
	return f
}

// TestCrossingAnnulus pins agent-docs/clearance-band-design.md §4.2: a plane
// with a hole of radius 3.2, crossed by a cylinder of radius 3 whose axis the
// oracle does not prove parallel to the normal. A tilt of 1e-12 rad (inside
// the oracle's undecided band) and of 0.01 rad (provably not parallel) cross
// in an ellipse of semi-major radius at most 3.0001, wholly inside the hole:
// the pair never crosses and the cell contributes nothing. A tilt of 0.6 rad
// crosses in an ellipse of semi-major radius 3.634, which leaves the hole, so
// the crossing stays undecided: reading the annulus's outer radius R/√(1 − s²)
// as R would exclude it. A hole of radius 3, which the cylinder touches, stays
// undecided.
func TestCrossingAnnulus(t *testing.T) {
	t.Parallel()
	for _, c := range []struct {
		name   string
		hole   float64
		tilt   float64
		unsure bool
	}{
		{"tilt 1e-12", 3.2, 1e-12, false},
		{"tilt 0.01", 3.2, 0.01, false},
		{"tilt 0.6", 3.2, 0.6, true},
		{"hole the cylinder touches", 3, 1e-12, true},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			axis, ok := r3.NewVec(math.Sin(c.tilt), 0, math.Cos(c.tilt)).Normalize()
			require.True(t, ok)
			plane := holedPlane(t, c.hole)
			pin := cylinderFace(r3.Vec{}, axis, 3, -5, 5)
			sink, _ := faceCell(t, plane, pin, 20)
			require.Equal(t, c.unsure, sink.Unsure)
			require.False(t, sink.Overlap)
			require.Empty(t, sink.Contribs, `an exclusion contributes no distance`)
		})
	}
	t.Run("axis too far from the normal", func(t *testing.T) {
		t.Parallel()
		// A tilt of 70 degrees puts |n·a| below one half: no annulus is read.
		axis := r3.NewVec(math.Sin(1.22), 0, math.Cos(1.22))
		sink, _ := faceCell(t, holedPlane(t, 3.2), cylinderFace(r3.Vec{}, axis, 3, -5, 5), 20)
		require.True(t, sink.Unsure)
	})
}
