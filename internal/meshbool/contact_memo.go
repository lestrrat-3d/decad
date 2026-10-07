package meshbool

// ContactMemo caches TriTriClassify's answer per facet-index pair for ONE
// operand pair, so the second ask a call reaches — the tangency gate
// (facesNearMiss) and the mesh pass (MeshBoolean) walk the same two prepared
// tessellations — is served from the store instead of recomputed.
// TriTriClassify is a pure function of the two facets' float corners, exact
// lifts and exact normals, all read-only fields of ma/mb, so a stored answer
// for (i, j) stays correct for as long as ma and mb do — which is exactly one
// evaluateBoolean call, the memo's whole lifetime. Binding ma/mb into the
// memo itself, rather than taking them per call, means no ask names an
// operand, so a caller cannot key an answer for one operand pair and read it
// back for another.
//
// A returned TriContact is shared: its p0/p1 endpoints and its sin2 are
// *big.Rat values a second caller reads, never a fresh copy. Every consumer
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
type ContactMemo struct {
	Ma, Mb *BoolMesh
	Cols   int
	Dense  []uint32
	Values []TriContact
	Sparse map[[2]int32]uint64
}

const (
	ContactMemoDensePairLimit    = 2_000_000
	ContactMemoDensePromoteRatio = 64
)

// NewContactMemo builds a memo scoped to one evaluateBoolean call over ma/mb.
func NewContactMemo(ma, mb *BoolMesh) *ContactMemo {
	return &ContactMemo{Ma: ma, Mb: mb, Cols: len(mb.Tris), Sparse: map[[2]int32]uint64{}}
}

func (c *ContactMemo) Lookup(i, j int) (TriContact, bool) {
	var status uint64
	var ok bool
	if c.Dense != nil {
		status = uint64(c.Dense[i*c.Cols+j])
		ok = status != 0
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
	if c.Dense != nil {
		index := i*c.Cols + j
		c.Dense[index] = uint32(status)
		return
	}
	key := [2]int32{int32(i), int32(j)}
	c.Sparse[key] = status
	c.PromoteDense()
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
