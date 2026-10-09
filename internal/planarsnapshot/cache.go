package planarsnapshot

import (
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// ConvexityEntry is a body's cached convexity certificate at its identity
// pose. Positive-determinant affine poses preserve the certificate.
type ConvexityEntry struct {
	ChordBits uint64
	ChordFree bool
	Convex    bool
}

// SnapshotEntry is a body's cached exact held snapshot at its identity query
// pose, or its refusal when OK is false. ChordFree snapshots serve every chord.
// Its slices are shared by readers and never written after the entry is stored.
type SnapshotEntry struct {
	ChordBits uint64
	ChordFree bool
	OK        bool
	Solid     planar.PlanarSolid
	Delta     proofarith.Dyadic
	Topology  *planar.PlanarTopology
}

// ServesChord reports whether the entry stands for the requested chord.
func (e *SnapshotEntry) ServesChord(bits uint64) bool {
	return e.ChordFree || e.ChordBits == bits
}
