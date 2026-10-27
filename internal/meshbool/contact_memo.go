package meshbool

// ContactMemo caches TriTriClassify's answer per facet-index pair for ONE
// operand pair, so the second ask a call reaches — the tangency gate
// (FacesNearMiss) and the mesh pass (MeshBoolean) walk the same two prepared
// tessellations — is served from the store instead of recomputed.
// TriTriClassify is a pure function of the two facets' float corners, exact
// lifts and exact normals, all read-only fields of ma/mb, so a stored answer
// for (i, j) stays correct for as long as ma and mb do — which is exactly one
// evaluateBoolean call, the memo's whole lifetime. Binding ma/mb into the
// memo itself, rather than taking them per call, means no ask names an
// operand, so a caller cannot key an answer for one operand pair and read it
// back for another.
//
// A returned TriContact is shared: its p0/p1 endpoints and its sin2 hold
// *big.Int values a second caller reads, never a fresh copy. Every consumer
// today only reads them, so this is safe, but no consumer may mutate one in
// place. ContactMemo is not safe for concurrent use; nothing today shares one
// across goroutines.
//
// The sparse store records a compact status instead of copying TriContact into
// every map bucket. Status 1 means ContactNone; status n+2 indexes values[n].
// Zero remains the dense table's absent marker. Once at least one of every 64
// possible pairs has been classified, a table of at most two million pairs is
// promoted to direct indexing. The table therefore occupies at most 8 MB,
// while larger or sparser operand products keep the compact map.
//
// A product too large for the dense table can still hold long runs of
// neighbouring columns, as the facets near each other on two faces do. Once at
// least ContactMemoBlockPromoteMin entries sit in the map and they average
// ContactMemoBlockMinFill or more per 64-column block they touch, the store
// moves to block mode. BlockIndex holds one uint32 per (row, block): 0 for a
// block never opened, k+1 for block k, whose 64 slots use the dense table's
// status encoding. Blocks live in Arena, a list of fixed-size chunks allocated
// one at a time, so growing the arena never holds two copies of it: its peak
// is its retained size plus one chunk. BlockIndex is capped at
// ContactMemoBlockIndexLimit entries and the arena at
// ContactMemoBlockArenaLimit blocks; a product whose index would be larger
// never enters block mode. A store that needs a new block once the arena is
// full goes to Sparse instead, so in block mode Sparse is the overflow map and
// grows as the map always did. Lookup consults it whenever the entry's block is
// absent.
//
// Outside the dense table, a key outside [0, rows) x [0, Cols) is kept in Far
// under its full int key, never truncated and never written to the block
// index or the arena, so it cannot alias an in-range key. Sparse therefore
// holds only in-range keys, which fit its int32 key because the operands have
// fewer than 2^31 facets; the dense table likewise holds only in-range keys.
// Block mode changes where a status is kept, never which status Lookup
// returns.
//
// Status values must fit a uint32, as the dense table already requires; the
// arena holds a status no larger than the dense table could. ContactMemo is
// not safe for concurrent use.
type ContactMemo struct {
	Ma, Mb *BoolMesh
	Cols   int
	Dense  []uint32
	Values []TriContact
	Sparse map[[2]int32]uint64

	BlockIndex  []uint32
	Arena       [][]uint32
	Far         map[[2]int]uint64
	arenaBlocks int
	blockRows   int
	blockCols   int
	nextTry     int
	// blockArenaLimit overrides ContactMemoBlockArenaLimit when non-zero.
	blockArenaLimit int
}

const (
	ContactMemoDensePairLimit    = 2_000_000
	ContactMemoDensePromoteRatio = 64

	// ContactMemoBlockIndexLimit caps the block index at 4 bytes per entry
	// (16 MB). ContactMemoBlockArenaLimit caps the arena at 256 bytes per
	// block (64 MB). ContactMemoBlockPromoteMin is the map size from which
	// block mode is tried, and ContactMemoBlockMinFill the average entries per
	// touched block it requires.
	ContactMemoBlockIndexLimit = 1 << 22
	ContactMemoBlockArenaLimit = 1 << 18
	ContactMemoBlockPromoteMin = 1 << 15
	ContactMemoBlockMinFill    = 8

	// contactMemoChunkBlocks is the number of blocks in one arena chunk
	// (256 KB).
	contactMemoChunkBlocks = 1 << 10

	contactMemoBlockShift = 6
	contactMemoBlockSize  = 1 << contactMemoBlockShift
)

// NewContactMemo builds a memo scoped to one evaluateBoolean call over ma/mb.
func NewContactMemo(ma, mb *BoolMesh) *ContactMemo {
	return &ContactMemo{Ma: ma, Mb: mb, Cols: len(mb.Tris), Sparse: map[[2]int32]uint64{}}
}

func (c *ContactMemo) Lookup(i, j int) (TriContact, bool) {
	var status uint64
	var ok bool
	if !c.inRange(i, j) {
		status, ok = c.Far[[2]int{i, j}]
	} else if c.Dense != nil {
		status = uint64(c.Dense[i*c.Cols+j])
		ok = status != 0
	} else if c.BlockIndex != nil {
		status, ok = c.lookupBlock(i, j)
	} else {
		status, ok = c.Sparse[[2]int32{int32(i), int32(j)}]
	}
	if !ok {
		return TriContact{}, false
	}
	if status == 1 {
		return TriContact{EdgeA: -1, EdgeB: -1}, true
	}
	return c.Values[status-2], true
}

func (c *ContactMemo) Store(i, j int, v TriContact) {
	status := uint64(1)
	if v.Kind != ContactNone {
		c.Values = append(c.Values, v)
		status = uint64(len(c.Values) + 1)
	}
	if !c.inRange(i, j) {
		if c.Far == nil {
			c.Far = map[[2]int]uint64{}
		}
		c.Far[[2]int{i, j}] = status
		return
	}
	if c.Dense != nil {
		index := i*c.Cols + j
		c.Dense[index] = uint32(status)
		return
	}
	key := [2]int32{int32(i), int32(j)}
	if c.BlockIndex != nil {
		if !c.storeBlock(i, j, uint32(status)) {
			c.Sparse[key] = status
		}
		return
	}
	c.Sparse[key] = status
	c.PromoteDense()
	c.promoteBlocks()
}

// inRange reports whether (i, j) lies inside the product of the two operands.
func (c *ContactMemo) inRange(i, j int) bool {
	return uint(i) < uint(len(c.Ma.Tris)) && uint(j) < uint(c.Cols)
}

func (c *ContactMemo) arenaLimit() int {
	if c.blockArenaLimit != 0 {
		return c.blockArenaLimit
	}
	return ContactMemoBlockArenaLimit
}

// blockSlot returns the BlockIndex position of (i, j), and false when (i, j)
// lies outside the product. Inside the product i < blockRows and
// j>>6 < blockCols, so the position is below blockRows*blockCols, which
// promoteBlocks keeps within ContactMemoBlockIndexLimit: no overflow, and no
// two keys share a position unless they share a row and a 64-column block.
func (c *ContactMemo) blockSlot(i, j int) (int, bool) {
	if uint(i) >= uint(c.blockRows) || uint(j) >= uint(c.Cols) {
		return 0, false
	}
	return i*c.blockCols + (j >> contactMemoBlockShift), true
}

func (c *ContactMemo) lookupBlock(i, j int) (uint64, bool) {
	if bi, in := c.blockSlot(i, j); in {
		if k := c.BlockIndex[bi]; k != 0 {
			status := c.slot(k, j)
			return uint64(*status), *status != 0
		}
	}
	if len(c.Sparse) == 0 {
		return 0, false
	}
	status, ok := c.Sparse[[2]int32{int32(i), int32(j)}]
	return status, ok
}

// storeBlock writes status into (i, j)'s block, opening the block while the
// arena has room. It reports false, having written nothing, when the key lies
// outside the product or its block is absent and the arena is full.
func (c *ContactMemo) storeBlock(i, j int, status uint32) bool {
	bi, in := c.blockSlot(i, j)
	if !in {
		return false
	}
	k := c.BlockIndex[bi]
	if k == 0 {
		if c.arenaBlocks >= c.arenaLimit() {
			return false
		}
		c.growArena()
		k = uint32(c.arenaBlocks)
		c.BlockIndex[bi] = k
	}
	*c.slot(k, j) = status
	return true
}

// slot returns column j's slot in block k (1-based). Block k-1 sits in chunk
// (k-1)/contactMemoChunkBlocks at the offset of its position in that chunk.
func (c *ContactMemo) slot(k uint32, j int) *uint32 {
	n := int(k - 1)
	chunk := c.Arena[n/contactMemoChunkBlocks]
	return &chunk[(n%contactMemoChunkBlocks)<<contactMemoBlockShift+j&(contactMemoBlockSize-1)]
}

// growArena opens one zeroed block, allocating a new chunk when the last one
// is full. A chunk is never larger than the blocks the limit still allows, so
// the arena never holds more than arenaLimit() blocks and never copies.
func (c *ContactMemo) growArena() {
	if c.arenaBlocks%contactMemoChunkBlocks == 0 {
		n := min(contactMemoChunkBlocks, c.arenaLimit()-c.arenaBlocks)
		c.Arena = append(c.Arena, make([]uint32, n<<contactMemoBlockShift))
	}
	c.arenaBlocks++
}

// promoteBlocks moves a large map into block mode when its in-range entries
// average at least ContactMemoBlockMinFill per block they touch and the
// blocks fit the arena. A refusal retries after the map has doubled.
func (c *ContactMemo) promoteBlocks() {
	if c.Sparse == nil || len(c.Sparse) < max(ContactMemoBlockPromoteMin, c.nextTry) {
		return
	}
	rows := len(c.Ma.Tris)
	blockCols := (c.Cols + contactMemoBlockSize - 1) >> contactMemoBlockShift
	if rows == 0 || blockCols == 0 || rows > ContactMemoBlockIndexLimit/blockCols {
		c.nextTry = int(^uint(0) >> 1)
		return
	}
	probe := ContactMemo{Cols: c.Cols, blockRows: rows, blockCols: blockCols}
	seen := make([]bool, rows*blockCols)
	blocks := 0
	for key := range c.Sparse {
		bi, in := probe.blockSlot(int(key[0]), int(key[1]))
		if !in || seen[bi] {
			continue
		}
		seen[bi] = true
		blocks++
	}
	if blocks*ContactMemoBlockMinFill > len(c.Sparse) || blocks > c.arenaLimit() {
		c.nextTry = 2 * len(c.Sparse)
		return
	}
	c.BlockIndex = make([]uint32, rows*blockCols)
	c.blockRows, c.blockCols = rows, blockCols
	old := c.Sparse
	c.Sparse = map[[2]int32]uint64{}
	for key, status := range old {
		if !c.storeBlock(int(key[0]), int(key[1]), uint32(status)) {
			c.Sparse[key] = status
		}
	}
}

func (c *ContactMemo) PromoteDense() {
	rows := len(c.Ma.Tris)
	if rows == 0 || c.Cols == 0 || rows > ContactMemoDensePairLimit/c.Cols {
		return
	}
	pairs := rows * c.Cols
	if len(c.Sparse)*ContactMemoDensePromoteRatio < pairs {
		return
	}
	c.Dense = make([]uint32, pairs)
	for key, status := range c.Sparse {
		c.Dense[int(key[0])*c.Cols+int(key[1])] = uint32(status)
	}
	c.Sparse = nil
}

// classify returns the exact classification of facet i of ma against facet j
// of mb, computing it on the first ask and replaying the stored answer on
// every later one. An error is never stored, so a later ask still retries.
func (c *ContactMemo) Classify(i, j int) (TriContact, error) {
	if v, ok := c.Lookup(i, j); ok {
		return v, nil
	}
	c.Ma.PrepareFloatNormal(i)
	c.Mb.PrepareFloatNormal(j)
	v, err := TriTriClassifyPrepared(TriCorners(c.Ma, i), TriCorners(c.Mb, j), XtriCorners(c.Ma, i), XtriCorners(c.Mb, j),
		c.Ma.Norms[i], c.Mb.Norms[j], c.Ma.Fnorms[i], c.Mb.Fnorms[j])
	if err != nil {
		return TriContact{}, err
	}
	c.Store(i, j, v)
	return v, nil
}
