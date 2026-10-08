package clearance

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestRevolveGapMeterChargesJoints pins RevolveWall.Joints: a cylinder wall
// at radius 2 that coalesced two recorded segments meeting at (500, 2 + j)
// about the u axis must carry an axis gap of at least j, since the carrier
// passes j from that recorded junction, and none when the junction sits on
// the carrier. The wall's two recorded ends match the carrier exactly, so
// only the joint can charge.
//
// Shown to fail: dropping the joint loop from revolveGapMeter.wall leaves the
// off-line junction at an axis gap of exactly zero.
func TestRevolveGapMeterChargesJoints(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	for _, j := range []float64{0, math.Ldexp(1, -33)} {
		first := survey2d.SegmentWalk{Kind: survey2d.WalkLine, StartU: 0, StartV: 2, EndU: 500, EndV: 2 + j}
		last := survey2d.SegmentWalk{Kind: survey2d.WalkLine, StartU: 500, StartV: 2 + j, EndU: 1000, EndV: 2}
		in := RevolveCarrierInput{
			Lift:      revolvemesh.RevolveLift{Frame: frame, DU: 1},
			Transform: r3.Identity(),
			Full:      true,
			Walls: []RevolveWall{{
				Walk: survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
					Kind: survey2d.WalkLine, StartU: 0, StartV: 2, EndU: 1000, EndV: 2,
					TanInU: 500, TanInV: j, TanOutU: 500, TanOutV: -j,
				}, Segs: []int{0, 1}},
				Kind:  revolveaxis.WallCylinder,
				First: first, Last: last,
				Joints: []RevolveJoint{{Z: 500, Rho: 2, Rec: revolvemesh.RecordedMeridian{U: 500, V: 2 + j}}},
			}},
		}
		gap := BuildRevolveCarriers(in).AxisGap
		if j == 0 {
			require.Zero(t, gap)
			continue
		}
		require.GreaterOrEqual(t, gap, j)
	}
}
