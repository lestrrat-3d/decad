package meshbool

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// memoPair builds a memo over operands of rows and cols facets. Only the facet
// counts matter: the memo never reads a facet while it stores and looks up.
func memoPair(rows, cols int) *ContactMemo {
	return NewContactMemo(&BoolMesh{Tris: make([][3]int, rows)}, &BoolMesh{Tris: make([][3]int, cols)})
}

// memoRef is the reference the memo must match for every key: a plain map from
// the key to the last value stored.
type memoRef map[[2]int]TriContact

func (m memoRef) store(memo *ContactMemo, i, j int, v TriContact) {
	memo.Store(i, j, v)
	m[[2]int{i, j}] = v
}

func (m memoRef) requireMatches(t *testing.T, memo *ContactMemo, i, j int) {
	t.Helper()
	got, ok := memo.Lookup(i, j)
	want, stored := m[[2]int{i, j}]
	if ok == stored && (!stored || got == want) {
		return
	}
	require.Equal(t, stored, ok, "(%d, %d) presence", i, j)
	if stored {
		require.Equal(t, want, got, "(%d, %d) value", i, j)
	}
}

func (m memoRef) requireAllMatch(t *testing.T, memo *ContactMemo) {
	t.Helper()
	for k := range m {
		m.requireMatches(t, memo, k[0], k[1])
	}
}

func testContact(r *rand.Rand, i, j int) TriContact {
	if r.IntN(50) == 0 {
		return TriContact{Kind: ContactSegment, EdgeA: i % 3, EdgeB: j % 3}
	}
	return TriContact{EdgeA: -1, EdgeB: -1}
}

// storeBand stores n neighbouring columns of row i from start, as the facets
// near each other on two faces do.
func (m memoRef) storeBand(r *rand.Rand, memo *ContactMemo, i, start, n int) {
	for j := start; j < start+n; j++ {
		m.store(memo, i, j, testContact(r, i, j))
	}
}

func TestContactMemoBlockModeMatchesReference(t *testing.T) {
	t.Parallel()
	const rows, cols = 3000, 1000 // 3M pairs: above ContactMemoDensePairLimit
	memo := memoPair(rows, cols)
	ref := memoRef{}
	r := rand.New(rand.NewPCG(11, 13))
	for i := range rows {
		ref.storeBand(r, memo, i, r.IntN(cols-40), 40)
	}
	require.NotNil(t, memo.BlockIndex, "a clustered store promotes to block mode")
	require.Nil(t, memo.Dense)
	require.Empty(t, memo.Sparse, "no entry overflows the default arena")
	ref.requireAllMatch(t, memo)
	for range 10000 {
		ref.requireMatches(t, memo, r.IntN(rows), r.IntN(cols))
	}
}

func TestContactMemoScatteredStoreStaysSparse(t *testing.T) {
	t.Parallel()
	const rows, cols = 3000, 1000
	memo := memoPair(rows, cols)
	ref := memoRef{}
	r := rand.New(rand.NewPCG(17, 19))
	for range 2 * ContactMemoBlockPromoteMin {
		ref.store(memo, r.IntN(rows), r.IntN(cols), TriContact{EdgeA: -1, EdgeB: -1})
	}
	require.Nil(t, memo.BlockIndex, "entries that fill under one in eight of their blocks keep the map")
	ref.requireAllMatch(t, memo)
}

func TestContactMemoBlockArenaOverflow(t *testing.T) {
	t.Parallel()
	const rows, cols = 3000, 1000
	const limit = 2500 // below the 3000 x 16 blocks stored, above the blocks present at promotion
	memo := memoPair(rows, cols)
	memo.blockArenaLimit = limit
	ref := memoRef{}
	r := rand.New(rand.NewPCG(11, 13))
	for i := range rows {
		ref.storeBand(r, memo, i, r.IntN(cols-40), 40)
	}
	require.NotNil(t, memo.BlockIndex)
	require.NotEmpty(t, memo.Sparse, "stores past the arena limit land in the overflow map")
	require.Equal(t, limit, memo.arenaBlocks)
	requireArenaBounded(t, memo)
	ref.requireAllMatch(t, memo)
	for range 10000 {
		ref.requireMatches(t, memo, r.IntN(rows), r.IntN(cols))
	}
	// Restoring an overflowed key keeps the overflow map as its home.
	for k := range memo.Sparse {
		ref.store(memo, int(k[0]), int(k[1]), TriContact{Kind: ContactSegment, EdgeA: 1, EdgeB: 2})
		break
	}
	ref.requireAllMatch(t, memo)
}

// TestContactMemoBlockIndexLimit pins the index cap: a product of exactly
// ContactMemoBlockIndexLimit blocks enters block mode and one more row stays a
// map, with every entry found either way.
func TestContactMemoBlockIndexLimit(t *testing.T) {
	t.Parallel()
	const cols = 1024 // 16 blocks per row
	const limitRows = ContactMemoBlockIndexLimit / (cols / contactMemoBlockSize)
	for _, tc := range []struct {
		name  string
		rows  int
		block bool
	}{
		{"at the limit", limitRows, true},
		{"one row over", limitRows + 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			memo := memoPair(tc.rows, cols)
			ref := memoRef{}
			r := rand.New(rand.NewPCG(23, 29))
			// Full blocks in the first and the last rows.
			for _, i := range []int{0, tc.rows - 1, tc.rows / 2} {
				for start := 0; start < cols; start += contactMemoBlockSize {
					ref.storeBand(r, memo, i, start, contactMemoBlockSize)
				}
			}
			for i := 1; len(ref) <= 2*ContactMemoBlockPromoteMin; i++ {
				ref.storeBand(r, memo, i, 0, cols)
			}
			require.Equal(t, tc.block, memo.BlockIndex != nil)
			ref.requireAllMatch(t, memo)
			ref.requireMatches(t, memo, tc.rows-1, cols-1)
			ref.requireMatches(t, memo, tc.rows-2, 0)
		})
	}
}

// TestContactMemoOutOfRangeKeysStayInTheMap pins that a key outside the product
// is never written to the arena: it cannot alias an in-range key or index out
// of range, and the memo returns what a map would.
func TestContactMemoOutOfRangeKeysStayInTheMap(t *testing.T) {
	t.Parallel()
	const rows, cols = 3000, 1000 // the last block of a row is 40 columns wide
	memo := memoPair(rows, cols)
	ref := memoRef{}
	r := rand.New(rand.NewPCG(31, 37))
	for i := range rows {
		ref.storeBand(r, memo, i, r.IntN(cols-40), 40)
	}
	require.NotNil(t, memo.BlockIndex)
	segment := TriContact{Kind: ContactSegment, EdgeA: 0, EdgeB: 1}
	// In every pair a key past an edge of the product, and the in-range key it
	// would alias under a missing range check.
	for _, k := range [][2]int{
		{-1, 0}, {0, -1}, {-1, -1}, {rows, 0}, {rows, cols - 1}, {0, cols}, {0, cols + 20}, {rows - 1, cols},
		{rows - 1, cols + 23}, {1 << 30, 5}, {5, 1 << 30}, {-(1 << 30), 5}, {5, -(1 << 30)}, {1<<31 - 1, 1<<31 - 1},
	} {
		ref.requireMatches(t, memo, k[0], k[1])
		ref.store(memo, k[0], k[1], segment)
		ref.requireMatches(t, memo, k[0], k[1])
	}
	ref.requireAllMatch(t, memo)
	// The aliasing candidates: a missing range check would store (-1, 0) at
	// (-1)*16+0 = -16 (panic) and (0, cols+20) in row 0's last block.
	for j := range cols {
		ref.requireMatches(t, memo, 0, j)
		ref.requireMatches(t, memo, rows-1, j)
	}
}

// TestContactMemoRandomSequencesMatchReference replays random store and lookup
// sequences against the reference map: a clustered phase that promotes the memo
// to block mode, then a phase over every row, with repeated stores, block-edge
// columns and absent keys. A small arena limit makes the second phase overflow
// into the map.
func TestContactMemoRandomSequencesMatchReference(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name       string
		rows, cols int
		overflow   bool // arena limit just above the clustered phase's blocks
		seed       uint64
	}{
		{"narrower than a block", 40000, 63, false, 43},
		{"exactly a block", 40000, 64, true, 47},
		{"one past a block", 40000, 65, true, 53},
		{"odd width", 5000, 997, false, 59},
		{"odd width, tiny arena", 5000, 997, true, 61},
		{"two blocks", 20000, 128, true, 67},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Greater(t, tc.rows*tc.cols, ContactMemoDensePairLimit, "dense mode would take this product")
			blockCols := (tc.cols + contactMemoBlockSize - 1) / contactMemoBlockSize
			hot := min(tc.rows, 2*ContactMemoBlockPromoteMin/tc.cols+1)
			memo := memoPair(tc.rows, tc.cols)
			if tc.overflow {
				memo.blockArenaLimit = hot*blockCols + 8
			}
			ref := memoRef{}
			r := rand.New(rand.NewPCG(tc.seed, 71))
			edges := []int{0, 1, 62, 63, 64, 65, 127, 128, tc.cols - 1, tc.cols - 2}
			rowLimit := hot
			far := farKeys(tc.rows, tc.cols)
			pick := func() (int, int) {
				i := r.IntN(rowLimit)
				if r.IntN(8) == 0 {
					i = []int{0, rowLimit - 1}[r.IntN(2)]
				}
				j := r.IntN(tc.cols)
				if e := edges[r.IntN(len(edges))]; e >= 0 && e < tc.cols && r.IntN(4) == 0 {
					j = e
				}
				return i, j
			}
			step := func() {
				switch r.IntN(5) {
				case 0: // an isolated key, sometimes one stored before
					i, j := pick()
					ref.store(memo, i, j, testContact(r, i, j))
				case 1: // a run along a row
					i, j := pick()
					ref.storeBand(r, memo, i, j, min(1+r.IntN(2*contactMemoBlockSize), tc.cols-j))
				case 2: // a far key, or an in-range key shifted by 2^32
					k := far[r.IntN(len(far))]
					i, j := pick()
					switch r.IntN(3) {
					case 0:
						k = [2]int{i + 1<<32, j}
					case 1:
						k = [2]int{i, j + 1<<32}
					}
					if r.IntN(2) == 0 {
						ref.store(memo, k[0], k[1], testContact(r, i, j))
					}
					ref.requireMatches(t, memo, k[0], k[1])
					ref.requireMatches(t, memo, i, j)
				default:
					ref.requireMatches(t, memo, r.IntN(rowLimit), r.IntN(tc.cols))
					i, j := pick()
					ref.requireMatches(t, memo, i, j)
				}
			}
			for n := 0; memo.BlockIndex == nil; n++ {
				require.Less(t, n, 400000, "the clustered phase never promoted")
				step()
			}
			ref.requireAllMatch(t, memo) // the state just after promotion
			rowLimit = tc.rows
			for range 60000 {
				step()
			}
			ref.requireAllMatch(t, memo)
			if tc.overflow {
				require.NotEmpty(t, memo.Sparse, "the second phase fills past the arena limit")
				requireArenaBounded(t, memo)
			}
		})
	}
}

// requireArenaBounded pins the arena's memory: the retained blocks stay within
// the limit, and no single allocation is larger than one chunk, so growth never
// holds a second copy of the arena.
func requireArenaBounded(t *testing.T, memo *ContactMemo) {
	t.Helper()
	total := 0
	for _, chunk := range memo.Arena {
		require.LessOrEqual(t, len(chunk), contactMemoChunkBlocks<<contactMemoBlockShift)
		require.Equal(t, len(chunk), cap(chunk))
		total += len(chunk)
	}
	require.LessOrEqual(t, total, memo.arenaLimit()<<contactMemoBlockShift)
	require.GreaterOrEqual(t, total, memo.arenaBlocks<<contactMemoBlockShift)
}

// TestContactMemoArenaGrowthNeverPassesTheLimit opens blocks one at a time
// from an empty arena: chunks are allocated one at a time and never past the
// limit, the limit's blocks open, and the next block is refused without a
// write.
func TestContactMemoArenaGrowthNeverPassesTheLimit(t *testing.T) {
	t.Parallel()
	const rows, cols, limit = 2600, 64, 2500 // one block per row; 2.4 chunks
	memo := memoPair(rows, cols)
	memo.BlockIndex = make([]uint32, rows)
	memo.blockRows, memo.blockCols = rows, 1
	memo.blockArenaLimit = limit
	for i := range rows {
		opened := memo.storeBlock(i, cols-1, uint32(i+2))
		require.Equal(t, i < limit, opened, "row %d", i)
		requireArenaBounded(t, memo)
	}
	require.Equal(t, limit, memo.arenaBlocks)
	require.Len(t, memo.Arena, 3)
	require.Len(t, memo.Arena[2], (limit-2*contactMemoChunkBlocks)<<contactMemoBlockShift)
	for i := range limit {
		status, ok := memo.lookupBlock(i, cols-1)
		require.True(t, ok)
		require.Equal(t, uint64(i+2), status)
		_, ok = memo.lookupBlock(i, 0)
		require.False(t, ok, "a block's other slots stay absent")
	}
	_, ok := memo.lookupBlock(limit, cols-1)
	require.False(t, ok)
}

// farKeys returns keys outside a rows x cols product, among them keys that
// wrap onto an in-range key when truncated to 32 bits.
func farKeys(rows, cols int) [][2]int {
	return [][2]int{
		{-1, 0}, {0, -1}, {-1, -1}, {rows, 0}, {0, cols}, {rows, cols - 1}, {rows - 1, cols}, {0, cols + 20},
		{1 << 30, 5}, {5, 1 << 30}, {-(1 << 30), 5}, {5, -(1 << 30)},
		{1<<31 - 1, 3}, {1 << 31, 3}, {3, 1<<31 - 1}, {3, 1 << 31}, {1<<31 + 1, 1<<31 + 1},
		{1 << 32, cols - 1}, {1<<32 + 7, 5}, {7, 1<<32 + 5}, {-(1 << 32), 5}, {5, -(1 << 32)},
		{math.MinInt64, math.MinInt64}, {math.MaxInt64, math.MaxInt64}, {math.MinInt64, 0}, {0, math.MaxInt64},
	}
}

// TestContactMemoFarKeysSurvivePromotion stores far keys before and after the
// block promotion, including the keys that alias in-range keys under 32-bit
// truncation, and requires every far key and its in-range alias to answer as a
// map keyed by the full key does.
func TestContactMemoFarKeysSurvivePromotion(t *testing.T) {
	t.Parallel()
	const rows, cols = 1000, 3000
	segment := TriContact{Kind: ContactSegment, EdgeA: 1, EdgeB: 2}
	other := TriContact{Kind: ContactSegment, EdgeA: 2, EdgeB: 0}
	memo := memoPair(rows, cols)
	ref := memoRef{}
	keys := farKeys(rows, cols)
	for n, k := range keys[:len(keys)/2] { // before promotion
		v := segment
		if n%2 == 0 {
			v = TriContact{EdgeA: -1, EdgeB: -1}
		}
		ref.store(memo, k[0], k[1], v)
	}
	r := rand.New(rand.NewPCG(73, 79))
	for i := range rows {
		ref.storeBand(r, memo, i, 0, 40)
	}
	require.NotNil(t, memo.BlockIndex, "the clustered fill promotes")
	for _, k := range keys[len(keys)/2:] { // after promotion
		ref.store(memo, k[0], k[1], other)
	}
	// The reviewed case: a far key whose 32-bit truncation is (0, 2000).
	ref.store(memo, 1<<32, 2000, segment)
	ref.requireMatches(t, memo, 0, 2000)
	ref.requireAllMatch(t, memo)
	for _, k := range keys {
		ref.requireMatches(t, memo, k[0], k[1])
		// Its in-range alias under truncation to 32 bits.
		if ai, aj := int(int32(k[0])), int(int32(k[1])); ai >= 0 && ai < rows && aj >= 0 && aj < cols {
			ref.requireMatches(t, memo, ai, aj)
		}
	}
	for j := range cols {
		ref.requireMatches(t, memo, 0, j)
	}
}

// TestContactMemoFarKeysInDenseTable pins the same rule once the dense table
// holds the in-range keys.
func TestContactMemoFarKeysInDenseTable(t *testing.T) {
	t.Parallel()
	const rows, cols = 500, 400
	memo := memoPair(rows, cols)
	ref := memoRef{}
	segment := TriContact{Kind: ContactSegment, EdgeA: 1, EdgeB: 2}
	ref.store(memo, 1<<32, 7, segment)
	r := rand.New(rand.NewPCG(83, 89))
	for i := range rows {
		ref.storeBand(r, memo, i, 0, 40)
	}
	require.NotNil(t, memo.Dense)
	for _, k := range farKeys(rows, cols) {
		ref.requireMatches(t, memo, k[0], k[1])
		ref.store(memo, k[0], k[1], segment)
	}
	ref.requireAllMatch(t, memo)
	ref.requireMatches(t, memo, 0, 7)
}
