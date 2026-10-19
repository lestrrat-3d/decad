package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestTiltedPrismPublishesBoundedVolume(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))
	require.NoError(t, err)
	world := sketch.NewWorld()
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 4, 3)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
	require.NoError(t, err)

	volume, err := body.Volume()
	require.NoError(t, err)
	want := big.NewRat(12, 1)
	got := new(big.Rat).SetFloat64(volume.Value.Base())
	bound := new(big.Rat).SetFloat64(volume.Bound.Base())
	require.NotNil(t, got)
	require.NotNil(t, bound)
	difference := new(big.Rat).Sub(got, want)
	difference.Abs(difference)
	require.LessOrEqual(t, difference.Cmp(bound), 0,
		"the published volume bound covers the rectangular prism's volume")
	require.LessOrEqual(t, volume.Bound.Base(), 1e-8)
}
