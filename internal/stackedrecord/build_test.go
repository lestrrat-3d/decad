package stackedrecord

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRegionPartIgnoresUnusedOverflowedSecondMoments(t *testing.T) {
	t.Parallel()
	const side = 1e100
	line := func(u0, v0, u1, v1 float64) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: u0, V: v0},
			End:   sectionrecord.Point2{U: u1, V: v1},
			TEnd:  1,
		}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(0, 0, side, 0),
		line(side, 0, side, side),
		line(side, side, 0, side),
		line(0, side, 0, 0),
	}}}
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	stack := Payload{
		Slabs: []Slab{{Regions: []momentinput.Profile{profile}, Z1: 2}},
		Frame: frame,
		Xform: r3.Identity(),
	}

	// The region's second moments exceed float64, but its area and first moments fit.
	part, err := regionPart(t.Context(), stack, stack.OuterPrism(), nil, nil, nil, 0, 0,
		freeform.NewFreeformWork(), nil, nil)
	require.NoError(t, err)
	require.InDelta(t, 1e200, part.area.Value, 1e185)
	require.InDelta(t, 5e99, part.centroid.X, 1e85)
	require.InDelta(t, 5e99, part.centroid.Y, 1e85)
}
