package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// LatticeKind names the lattice an [Infill] fills a body with.
type LatticeKind int

const (
	// GyroidInfill is the gyroid of [GyroidLattice].
	GyroidInfill LatticeKind = iota
	// SchwarzPInfill is the Schwarz primitive lattice of [SchwarzPLattice].
	SchwarzPInfill
	// DiamondInfill is the diamond lattice of [DiamondLattice].
	DiamondInfill
)

// String renders the kind for diagnostics.
func (k LatticeKind) String() string {
	switch k {
	case GyroidInfill:
		return "GyroidInfill"
	case SchwarzPInfill:
		return "SchwarzPInfill"
	case DiamondInfill:
		return "DiamondInfill"
	default:
		return fmt.Sprintf("LatticeKind(%d)", int(k))
	}
}

// InfillSpec describes the lattice an [Infill] fills a body with: its kind, the length it repeats at
// (Cell), its dimensionless wall thickness (Level) and its samples to a cell (Resolution), each as in
// [LatticeSpec].
type InfillSpec struct {
	Kind        LatticeKind
	Cell, Level units.Value
	Resolution  int
}

// Infill fills b with a lattice and returns the lattice material inside it: the lattice over a box
// that encloses b by a margin of a quarter cell, made a solid by [SolidFromMesh] and intersected with b
// (agent-docs/lattice-design.md §5). b and the lattice are consumed, as by [Intersect], and the result
// replaces them in b's document. The lattice is the part's whole interior; to keep a skin, [Shell]
// the part and [Union] the infill into its cavity.
//
// b MUST be a live solid with a proven bounding box. The lattice is cut to b by the mesh Boolean, so
// the result is a faceted body whose volume reads inside the bound the Boolean proves. The lattice
// mesh must be one [SolidFromMesh] takes (a closed mesh of at most 65,528 triangles), so a coarser
// Resolution or a larger Cell admits a bigger part, and a lattice too fine for that is ErrUnsupported.
// The spec's errors are those of the lattice builders, an unknown Kind is ErrDegenerate and a nil
// context or body ErrDegenerate. An intersection that leaves nothing is the Boolean's own error.
func Infill(ctx context.Context, b *Body, spec InfillSpec) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an infill`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: Infill needs a solid body`, ErrNotSolid)
	}
	cell, err := spec.Cell.In(units.Millimeter)
	if err != nil {
		return nil, fmt.Errorf(`%w: the infill cell must be a length: %s`, ErrUnitKind, err)
	}
	if math.IsNaN(cell) || math.IsInf(cell, 0) || cell <= 0 {
		return nil, fmt.Errorf(`%w: the infill cell must be a positive finite length`, ErrDegenerate)
	}
	box, err := b.Bounds()
	if err != nil {
		return nil, err
	}
	margin := cell / 4
	ls := LatticeSpec{
		Min:  box.Min.Sub(r3.NewVec(margin, margin, margin)),
		Max:  box.Max.Add(r3.NewVec(margin, margin, margin)),
		Cell: spec.Cell, Level: spec.Level, Resolution: spec.Resolution,
	}
	var build func(context.Context, LatticeSpec) (*MeshBody, error)
	switch spec.Kind {
	case GyroidInfill:
		build = GyroidLattice
	case SchwarzPInfill:
		build = SchwarzPLattice
	case DiamondInfill:
		build = DiamondLattice
	default:
		return nil, fmt.Errorf(`%w: unknown infill lattice %s`, ErrDegenerate, spec.Kind)
	}
	mesh, err := build(ctx, ls)
	if err != nil {
		return nil, err
	}
	lattice, err := SolidFromMesh(ctx, documentHandle(b.n.Doc), mesh)
	if err != nil {
		return nil, fmt.Errorf("%w: the infill lattice is not a solid the evaluator can prove: %s", ErrUnsupported, err)
	}
	return Intersect(ctx, b, lattice)
}
