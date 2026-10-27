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
	// NeoviusInfill is the Neovius lattice of [NeoviusLattice].
	NeoviusInfill
	// IWPInfill is the IWP lattice of [IWPLattice].
	IWPInfill
	// FischerKochSInfill is the Fischer-Koch S lattice of [FischerKochSLattice].
	FischerKochSInfill
	// LidinoidInfill is the Lidinoid lattice of [LidinoidLattice].
	LidinoidInfill
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
	case NeoviusInfill:
		return "NeoviusInfill"
	case IWPInfill:
		return "IWPInfill"
	case FischerKochSInfill:
		return "FischerKochSInfill"
	case LidinoidInfill:
		return "LidinoidInfill"
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
	// Skin is the thickness of a solid skin the infill keeps round the part, a positive length. The zero value
	// keeps none, and the lattice is the part's whole interior.
	Skin units.Value
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
	if spec.Skin == (units.Value{}) {
		return infillInterior(ctx, b, spec)
	}
	return infillWithSkin(ctx, b, spec)
}

// infillWithSkin is Infill for a spec with a Skin: the part's closed [Shell] of the skin's thickness joined by [Union]
// to the lattice cut to the part, each built from its own copy of b. b is consumed when the whole join has built, and a
// refusal at any step retires the copies made so far and leaves b live.
func infillWithSkin(ctx context.Context, b *Body, spec InfillSpec) (result *Body, retErr error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an infill`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	var temporaries []*Body
	defer func() {
		if retErr != nil {
			removeTools(temporaries)
		}
	}()
	skinSource, err := Copy(ctx, b)
	if err != nil {
		return nil, err
	}
	temporaries = append(temporaries, skinSource)
	fillSource, err := Copy(ctx, b)
	if err != nil {
		return nil, err
	}
	temporaries = append(temporaries, fillSource)
	skin, err := Shell(ctx, skinSource, nil, spec.Skin, WithNoOpenings())
	if err != nil {
		return nil, fmt.Errorf("decad: the infill skin: %w", err)
	}
	temporaries = append(temporaries, skin)
	interior := spec
	interior.Skin = units.Value{}
	fill, err := infillInterior(ctx, fillSource, interior)
	if err != nil {
		return nil, err
	}
	temporaries = append(temporaries, fill)
	joined, err := Union(ctx, skin, fill)
	if err != nil {
		return nil, err
	}
	removeTools([]*Body{b})
	return joined, nil
}

// infillInterior fills the whole of b with the lattice (Infill without a Skin).
func infillInterior(ctx context.Context, b *Body, spec InfillSpec) (*Body, error) {
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
	case NeoviusInfill:
		build = NeoviusLattice
	case IWPInfill:
		build = IWPLattice
	case FischerKochSInfill:
		build = FischerKochSLattice
	case LidinoidInfill:
		build = LidinoidLattice
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
