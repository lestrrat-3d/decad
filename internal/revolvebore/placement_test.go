package revolvebore_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvebore"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestMapToolRequiresExactFlushCaps(t *testing.T) {
	meridianFrame, err := r3.NewFrame(r3.Vec{}, r3.Vec{Z: 1}, r3.Vec{X: 1})
	require.NoError(t, err)
	rp := revolveplan.Payload{Frame: meridianFrame, Ax: revolveaxis.Frame{DU: 1}, Xform: r3.Identity()}
	mer := revolvebore.Meridian{Low: 0, High: 32, MinRadius: 8}
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		v      r3.Vec
		end    float64
		ok     bool
	}{
		{"along", r3.Vec{}, r3.Vec{Y: 1}, 32, true},
		{"against", r3.Vec{Z: 32}, r3.Vec{Y: -1}, 32, true},
		{"off axis", r3.Vec{X: 1}, r3.Vec{Y: 1}, 32, false},
		{"tilted", r3.Vec{}, r3.Vec{Y: math.Sqrt(0.5), Z: math.Sqrt(0.5)}, 32, false},
		{"short", r3.Vec{}, r3.Vec{Y: 1}, math.Nextafter(32, 0), false},
		{"long", r3.Vec{}, r3.Vec{Y: 1}, math.Nextafter(32, 33), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, r3.Vec{X: 1}, tc.v)
			require.NoError(t, err)
			tool := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z1: tc.end,
				Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
					sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 1}, End: sectionrecord.Point2{V: 1}, TEnd: 1},
				}}}}
			mapped, ok, err := revolvebore.MapTool(t.Context(), rp, mer, tool)
			require.NoError(t, err)
			require.Equal(t, tc.ok, ok)
			if ok {
				require.Equal(t, 0.0, mapped.Z0)
				require.Equal(t, 32.0, mapped.Z1)
			}
		})
	}
}
