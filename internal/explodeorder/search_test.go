package explodeorder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/explodeorder"
	"github.com/stretchr/testify/require"
)

// scorer prefers direction 0, then the lower trailing value given per part.
type scorer struct{ trailing map[int]float64 }

func (scorer) Joint(d int) bool                           { return d == 0 }
func (scorer) Outward(ms []int, d int) bool               { return d == 0 }
func (scorer) Travel(ms []int, d int, rem []bool) float64 { return 0 }
func (scorer) Positive(d int) bool                        { return d == 0 }
func (s scorer) Trailing(ms []int, d int) float64         { return s.trailing[ms[0]] }

// input builds an Input over n parts, part 0 the ground and one direction. block[p]
// lists the parts blocking p.
func input(n int, block map[int][]int) explodeorder.Input {
	b := make([][]int, n)
	for p, l := range block {
		b[p] = l
	}
	in := explodeorder.Input{N: n, Ground: 0, Gone: make([]bool, n), Blockers: [][][]int{b},
		Group: make([]int, n), After: make([][]int, n), Fixed: make([]int, n), Score: scorer{}}
	for i := range n {
		in.Group[i], in.Fixed[i] = -1, -1
	}
	return in
}

func members(rounds [][]explodeorder.Unit) [][][]int {
	var out [][][]int
	for _, r := range rounds {
		var row [][]int
		for _, u := range r {
			row = append(row, u.Members)
		}
		out = append(out, row)
	}
	return out
}

func TestChainLeavesInRounds(t *testing.T) {
	// 3 is blocked by 2, 2 by 1, 1 by nothing but the ground.
	r, err := explodeorder.Search(input(4, map[int][]int{3: {2}, 2: {1}}))
	require.NoError(t, err)
	require.Equal(t, [][][]int{{{1}}, {{2}}, {{3}}}, members(r))
}

func TestCycleOfThreeGroupsAndTerminates(t *testing.T) {
	in := input(5, map[int][]int{1: {2}, 2: {3}, 3: {1}, 4: {1}})
	r, err := explodeorder.Search(in)
	require.NoError(t, err)
	require.Equal(t, [][][]int{{{1, 2, 3}}, {{4}}}, members(r))
}

func TestTrailingFaceOrdersARound(t *testing.T) {
	in := input(4, nil)
	in.Score = scorer{trailing: map[int]float64{1: 5, 2: 1, 3: 3}}
	r, err := explodeorder.Search(in)
	require.NoError(t, err)
	// The scorer reads the first member, so each part's value is its own.
	require.Equal(t, [][][]int{{{2}, {3}, {1}}}, members(r))
}

func TestGroundCycleIsNoDirection(t *testing.T) {
	in := input(3, map[int][]int{1: {0}, 0: {1}, 2: {1}})
	_, err := explodeorder.Search(in)
	var nd *explodeorder.NoDirectionError
	require.ErrorAs(t, err, &nd)
	require.Equal(t, 1, nd.Part)
	require.Equal(t, []explodeorder.DirBlock{{Dir: 0, By: []int{0}}}, nd.Blocked)
	require.Equal(t, 1, nd.Others)
}

func TestAfterCycleIsConflict(t *testing.T) {
	in := input(3, nil)
	in.After[1], in.After[2] = []int{2}, []int{1}
	_, err := explodeorder.Search(in)
	var cf *explodeorder.ConflictError
	require.ErrorAs(t, err, &cf)
	require.Equal(t, explodeorder.OrderCycle, cf.Kind)
}

func TestFreePartLeavesBeforeWhatItBlocks(t *testing.T) {
	// 1 is a cover blocking 2, 2 a planet blocking 3, the carrier; 4 is a pin, free at once, that also
	// blocks the carrier. The pin leaves in the round just before the carrier, with the planet.
	r, err := explodeorder.Search(input(5, map[int][]int{2: {1}, 3: {2, 4}}))
	require.NoError(t, err)
	require.Equal(t, [][][]int{{{1}}, {{2}, {4}}, {{3}}}, members(r))
}

func TestPartBlockingNothingLeavesLast(t *testing.T) {
	// 3 blocks nothing and leaves with 2, the last part; the round it was found free in empties and goes.
	r, err := explodeorder.Search(input(4, map[int][]int{2: {1}}))
	require.NoError(t, err)
	require.Equal(t, [][][]int{{{1}}, {{2}, {3}}}, members(r))
}

func TestAfterHoldsALatePart(t *testing.T) {
	// 3 blocks nothing, but 2 must leave after it, so it stays one round ahead of 2.
	in := input(4, map[int][]int{2: {1}})
	in.After[2] = []int{3}
	r, err := explodeorder.Search(in)
	require.NoError(t, err)
	require.Equal(t, [][][]int{{{1}, {3}}, {{2}}}, members(r))
}

func TestWaitingPartKeepsItsRound(t *testing.T) {
	// 4 blocks nothing but waits for 1, so it stays in the round just after 1 instead of moving to the
	// last round.
	r, err := explodeorder.Search(input(5, map[int][]int{2: {1}, 3: {2}, 4: {1}}))
	require.NoError(t, err)
	require.Equal(t, [][][]int{{{1}}, {{2}, {4}}, {{3}}}, members(r))
}

func TestGroupMovesAsOne(t *testing.T) {
	// 1 and 2 block each other and leave as a group; 3 is free but blocks 4, which waits for the group.
	r, err := explodeorder.Search(input(5, map[int][]int{1: {2}, 2: {1}, 4: {1, 3}}))
	require.NoError(t, err)
	require.Equal(t, [][][]int{{{1, 2}, {3}}, {{4}}}, members(r))
}

func TestGoneNeverBlocks(t *testing.T) {
	in := input(3, map[int][]int{1: {2}})
	in.Gone[2] = true
	r, err := explodeorder.Search(in)
	require.NoError(t, err)
	require.Equal(t, [][][]int{{{1}}}, members(r))
}
