package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestDirectLoftCircleGateRejectsEqualBaseDifferentRecord(t *testing.T) {
	bottom, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	top, err := r3.NewFrame(r3.NewVec(0, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	circle := circleSeg{Radius: units.Millimeters(10), TStart: 1, TEnd: 0}
	other := circle
	other.Radius = units.Centimeters(1)
	require.Equal(t, circle.Radius.Base(), other.Radius.Base())
	pl := loftPayload{
		profile0: profileRecord{Holes: []loopRecord{{Segments: []curveSegment{circle}}}},
		profile1: profileRecord{Holes: []loopRecord{{Segments: []curveSegment{circle}}}},
		frame0:   bottom, frame1: top,
		xform: r3.Identity(), authenticatedReconstruction: true,
	}
	require.Equal(t, loftCircleWallCylinder, directLoftCircularHole(pl))
	pl.profile1.Holes[0].Segments[0] = other
	require.Equal(t, loftCircleWallNone, directLoftCircularHole(pl))
}

func TestDirectLoftRuledCircleWallHasBodyNode(t *testing.T) {
	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{world.XY(), upper} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(-5, -5, 5, 5)
		s.Fix(rect.A)
		x := 0.0
		if i == 1 {
			x = 1
		}
		center := s.CreatePoint(x, 0)
		s.Fix(center)
		s.CreateCircle(center, 1)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	body, err := New().Loft(t.Context(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	var ruled *Face
	for _, face := range body.Faces() {
		if _, ok := face.Surface().(NURBSSurface); ok {
			require.Nil(t, ruled)
			ruled = face
		}
	}
	require.NotNil(t, ruled)
	require.Same(t, bodyNode(body), ruled.n.Body)
	require.Same(t, body, bodyHandle(ruled.n.Body))
}

// The three homogeneous Bernstein controls in loftCircularWallSurface's
// source-net construction give X=1-t², Y=2t and W=1+t². The polynomial
// identity proves every parameter lies on the circle, not just a sample.
func TestRuledCircleQuarterControlNet(t *testing.T) {
	multiply := func(a, b []int64) []int64 {
		out := make([]int64, len(a)+len(b)-1)
		for i, ai := range a {
			for j, bj := range b {
				out[i+j] += ai * bj
			}
		}
		return out
	}
	x, y, w := []int64{1, 0, -1}, []int64{0, 2}, []int64{1, 0, 1}
	x2, y2 := multiply(x, x), multiply(y, y)
	for i := range y2 {
		x2[i] += y2[i]
	}
	require.Equal(t, multiply(w, w), x2)

	rat := func(n, d int64) *big.Rat { return new(big.Rat).SetFrac64(n, d) }
	one := rat(1, 1)
	for _, angle := range []*big.Rat{rat(0, 1), rat(1, 3), rat(1, 2), rat(1, 1)} {
		t2 := new(big.Rat).Mul(angle, angle)
		xUnit := new(big.Rat).Quo(new(big.Rat).Sub(one, t2), new(big.Rat).Add(one, t2))
		yUnit := new(big.Rat).Quo(new(big.Rat).Mul(rat(2, 1), angle), new(big.Rat).Add(one, t2))
		for _, height := range []*big.Rat{rat(0, 1), rat(1, 4), rat(1, 1)} {
			// Interpolate circles (center,radius)=(0,0,1) and (1,1,2).
			center, radius := height, new(big.Rat).Add(one, height)
			xWorld := new(big.Rat).Add(center, new(big.Rat).Mul(radius, xUnit))
			yWorld := new(big.Rat).Add(center, new(big.Rat).Mul(radius, yUnit))
			dx := new(big.Rat).Sub(xWorld, center)
			dy := new(big.Rat).Sub(yWorld, center)
			distance2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
			require.Zero(t, distance2.Cmp(new(big.Rat).Mul(radius, radius)))
		}
	}
}
