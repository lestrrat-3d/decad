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

// filletedCapCut builds a rounded hexagonal prism with both cap loops filleted and cuts a round bore through it.
func filletedCapCut(t *testing.T) *decad.Body {
	t.Helper()
	ctx := t.Context()
	doc := decad.New()
	s := shortStepSketch(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	addRoundedHex(t, s, 10, 2)
	body, err := decad.Extrude(ctx, doc, s, shortStepProfile(t, s), decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
	require.NoError(t, err)
	loops := decad.Edges(decad.OuterLoopOf(decad.CapStart(body))).Or(decad.OuterLoopOf(decad.CapEnd(body)))
	body, err = decad.Fillet(ctx, body, loops, units.Millimeters(1))
	require.NoError(t, err)
	c := shortStepSketch(t, r3.Vec{Z: -1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	c.AddConstraint(sketch.NewRadius(c.CreateCircle(c.Origin(), 3), 3))
	tool, err := decad.Extrude(ctx, doc, c, shortStepProfile(t, c), decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	out, err := decad.Cut(ctx, body, tool)
	require.NoError(t, err)
	return out
}

// TestFilletBandCutBoundRepeats builds the same filleted cut in fresh documents and requires the result's volume and
// centroid bounds to be the same bits every time. The fillet band's occupied-volume terms used to be summed in Go's
// map order, so the bounds changed in their last bit from run to run.
func TestFilletBandCutBoundRepeats(t *testing.T) {
	type bits struct{ vol, volBound, cenBound uint64 }
	read := func(b *decad.Body) bits {
		vol, err := b.Volume()
		require.NoError(t, err)
		cen, err := b.Centroid()
		require.NoError(t, err)
		return bits{math.Float64bits(vol.Value.Base()), math.Float64bits(vol.Bound.Base()), math.Float64bits(cen.Bound.Base())}
	}
	want := read(filletedCapCut(t))
	for i := range 15 {
		require.Equal(t, want, read(filletedCapCut(t)), "run %d", i+2)
	}
}
