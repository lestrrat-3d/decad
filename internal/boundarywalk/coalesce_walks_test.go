package boundarywalk

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// lineWalk is a straight walk between two points, its tangent the float
// difference of its ends, as a recorded line's walk carries.
func lineWalk(i int, su, sv, eu, ev float64) survey2d.SideWalk {
	return survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		Kind:   survey2d.WalkLine,
		StartU: su, StartV: sv, EndU: eu, EndV: ev,
		TanInU: eu - su, TanInV: ev - sv, TanOutU: eu - su, TanOutV: ev - sv,
	}, Segs: []int{i}}
}

// TestCoalesceWalksMergesOnlyExactlyCollinear pins exactlyCollinear: two line
// walks merge only when their chords are parallel over the rationals, they
// meet at one float point, and neither side's bound on that point reaches
// off the line. Every pair below passes the 1e-12 tangent test, so each
// refusal is the exact test's alone.
//
// Shown to fail: with exactlyCollinear answering true, the kink, the bound
// across the line, the bound on a tilted line and the split junction all
// merge.
func TestCoalesceWalksMergesOnlyExactlyCollinear(t *testing.T) {
	t.Parallel()
	k := math.Ldexp(1, -33)
	along := lineWalk(1, 0.6, 0, 1.6, 0)
	along.StartBound = proofbound.WalkEndBound{U: 6e-17}
	across := lineWalk(1, 0.6, 0, 1.6, 0)
	across.StartBound = proofbound.WalkEndBound{V: 6e-17}
	tilted := lineWalk(1, 1, 3, 3, 9)
	tilted.StartBound = proofbound.WalkEndBound{U: 6e-17}
	cases := []struct {
		name  string
		walks []survey2d.SideWalk
		want  int
	}{
		{"diagonal split", []survey2d.SideWalk{lineWalk(0, 0, 0, 1, 3), lineWalk(1, 1, 3, 3, 9)}, 1},
		{"kink", []survey2d.SideWalk{lineWalk(0, 0, 0, 500, -k), lineWalk(1, 500, -k, 1000, 0)}, 2},
		{"bound along the line", []survey2d.SideWalk{lineWalk(0, 0.1, 0, 0.6, 0), along}, 1},
		{"bound across the line", []survey2d.SideWalk{lineWalk(0, 0.1, 0, 0.6, 0), across}, 2},
		{"bound on a tilted line", []survey2d.SideWalk{lineWalk(0, 0, 0, 1, 3), tilted}, 2},
		{"split junction", []survey2d.SideWalk{lineWalk(0, 0, 0, 1, 0), lineWalk(1, math.Nextafter(1, 2), 0, 2, 0)}, 2},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			out, err := CoalesceChainWalksContext(t.Context(), tc.walks)
			require.NoError(t, err)
			require.Len(t, out, tc.want)
			if tc.want == 1 {
				first, last := tc.walks[0], tc.walks[len(tc.walks)-1]
				require.Equal(t, [4]float64{first.StartU, first.StartV, last.EndU, last.EndV},
					[4]float64{out[0].StartU, out[0].StartV, out[0].EndU, out[0].EndV})
				require.Equal(t, []int{0, 1}, out[0].Segs)
			}
		})
	}
}
