package sweeprecord

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// joinRims builds one span's rim pairs over posts at x = 0, 1, 2: wall j's
// rim runs from post j to post j+1, and each pair's start and end rim share
// those posts. shared reports whether the two walls share their middle post.
func joinRims(roles [][]string, shared bool) []chainRimPair {
	posts := []*topo.Vertex{
		{Position: r3.NewVec(0, 0, 0)},
		{Position: r3.NewVec(1, 0, 0)},
		{Position: r3.NewVec(2, 0, 0)},
	}
	middle := posts[1]
	if !shared {
		middle = &topo.Vertex{Position: posts[1].Position}
	}
	out := make([]chainRimPair, len(roles))
	for j := range roles {
		start, end := posts[j], posts[j+1]
		if j == 1 {
			start = middle
		}
		edge := &topo.Edge{Start: start, End: end}
		out[j] = chainRimPair{start: edge, end: edge, roles: roles[j]}
	}
	return out
}

func TestSewChainJoinRefusals(t *testing.T) {
	t.Parallel()
	two := [][]string{{"side(0,0)"}, {"side(0,1)"}}
	for _, tc := range []struct {
		name     string
		from, to []chainRimPair
		want     string
	}{
		{
			name: "wall counts differ",
			from: joinRims(two, true), to: joinRims(two[:1], true),
			want: "spans 3 and 4 have 2 and 1 source walls",
		},
		{
			name: "segment roles differ",
			from: joinRims(two, true), to: joinRims([][]string{{"side(0,0)"}, {"side(0,2)"}}, true),
			want: "join 3 wall 1 does not pair one recorded segment set",
		},
		{
			name: "one post pairs with two",
			from: joinRims(two, true), to: joinRims(two, false),
			want: "join 3 pairs one rim post with two",
		},
		{
			name: "two posts pair with one",
			from: joinRims(two, false), to: joinRims(two, true),
			want: "join 3 pairs two rim posts with one",
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			sewn := make(map[*topo.Edge]struct{})
			err := sewChainJoin(t.Context(), &topo.Body{}, tc.from, tc.to, 3, sewn)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
			require.Empty(t, sewn)
		})
	}
}

func TestSewChainJoinPairsByIndex(t *testing.T) {
	t.Parallel()
	two := [][]string{{"side(0,0)"}, {"side(0,1)"}}
	from, to := joinRims(two, true), joinRims(two, true)
	sewn := make(map[*topo.Edge]struct{})
	require.NoError(t, sewChainJoin(t.Context(), &topo.Body{}, from, to, 0, sewn))
	require.Len(t, sewn, 2)
	for j := range to {
		require.Contains(t, sewn, to[j].start)
	}
}
