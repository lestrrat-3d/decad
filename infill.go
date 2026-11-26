package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/lattice"
	"github.com/lestrrat-3d/decad/internal/modifyoption"
	"github.com/lestrrat-3d/decad/internal/opcommand"
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
	// SplitPInfill is the Split P lattice of [SplitPLattice].
	SplitPInfill
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
	case SplitPInfill:
		return "SplitPInfill"
	default:
		return fmt.Sprintf("LatticeKind(%d)", int(k))
	}
}

// infillSpec is the request an [InfillCommand] hands to the implementation: the
// constructor arguments and the methods that were called.
type infillSpec struct {
	kind        LatticeKind
	cell, level units.Value
	resolution  int
	skin        units.Value
	skinSet     bool
	// resolutionSubject and skinSubject name the method in a refusal of its value.
	resolutionSubject, skinSubject string
}

// Infill returns the command that fills b with a lattice and returns the lattice
// material inside it: the lattice over a box that encloses b by a margin of a
// quarter cell, made a solid by [SolidFromMesh] and intersected with b
// (agent-docs/lattice-design.md §5). b and the lattice are consumed, as by
// [Intersect], and the result replaces them in b's document. The lattice is the
// part's whole interior unless [InfillCommand.Skin] keeps a solid skin round it.
//
// kind names the lattice, cell is the length it repeats at and level its
// dimensionless wall thickness, each as in [LatticeSpec]. [InfillCommand.Resolution]
// sets the samples to a cell, which is 16 unless called.
//
// b MUST be a live solid with a proven bounding box. The lattice is cut to b by the
// mesh Boolean, so the result is a faceted body whose volume reads inside the bound
// the Boolean proves. The lattice mesh must be one [SolidFromMesh] takes (a closed
// mesh of at most 65,528 triangles), so a coarser resolution or a larger cell
// admits a bigger part, and a lattice too fine for that is ErrUnsupported. The
// errors of the values are those of the lattice builders, an unknown kind is
// ErrDegenerate and a nil context or body ErrDegenerate. An intersection that
// leaves nothing is the Boolean's own error. An error carries [OperationInfill],
// including the refusals of the nested [Shell], except a bare [BooleanError] of the
// nested [Union] or [Intersect], which keeps [OperationIntersect] or
// [OperationUnion]. A refusal of the nested Union or Intersect that
// names an operand names it `operand`.
func Infill(b *Body, kind LatticeKind, cell, level units.Value) *InfillCommand {
	c := &InfillCommand{b: b, spec: infillSpec{kind: kind, cell: cell, level: level}}
	if b == nil {
		c.state.Record("body", errNilBody)
	}
	return c
}

// InfillCommand fills a body with a lattice. Build it with [Infill].
type InfillCommand struct {
	state opcommand.State // consumed flag and first recorded fault
	b     *Body           // stored as given
	spec  infillSpec      // the constructor arguments and the methods called
}

// Resolution sets the samples to a cell of the lattice along each axis, from 4 up,
// as [LatticeSpec].Resolution does. A count of zero takes 16, not the value zero.
// A count below 4 is ErrDegenerate, returned by Do naming Resolution as the
// subject. A second call replaces the first.
func (c *InfillCommand) Resolution(samples int) *InfillCommand {
	c.spec.resolution = samples
	return c
}

// Skin keeps a solid skin of the given thickness round the part, a positive
// length, and the lattice fills what is inside it: the part's closed [Shell] of
// that thickness joined by [Union] to the lattice cut to the part. A thickness the
// shell refuses is returned by Do naming Skin as the subject. A second call
// replaces the first.
func (c *InfillCommand) Skin(thickness units.Value) *InfillCommand {
	c.spec.skin, c.spec.skinSet = thickness, true
	return c
}

// Do fills the body and retires it, or returns the error of the first gate that
// refuses, leaving the body live and the document unchanged. A nil ctx is
// ErrDegenerate, and a second call returns ErrConsumed.
func (c *InfillCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationInfill, c.run)
}

func (c *InfillCommand) run(ctx context.Context) (result *Body, retErr error) {
	if c.b == nil || c.b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	spec := c.spec
	spec.resolutionSubject, spec.skinSubject = "Resolution", "Skin"
	// The lattice body and the copies the skin route makes are published before the
	// result, so a failed or canceled call restores the document to what it found
	// (agent-docs/command-api-design.md §9). The snapshot comes before the gates,
	// which sit in infillInterior behind the skin route's copies, so a refusal
	// costs one snapshot.
	d := documentHandle(c.b.n.Doc)
	snap := d.snapshot(ctx)
	defer func() {
		if retErr != nil {
			d.restore(snap)
		}
	}()
	if !spec.skinSet {
		return infillInterior(ctx, c.b, spec)
	}
	return infillWithSkin(ctx, c.b, spec)
}

// infillWithSkin is Infill for a spec with a skin: the part's closed [Shell] of the skin's thickness joined by [Union]
// to the lattice cut to the part, each built from its own copy of b. b is consumed when the whole join has built, and a
// refusal at any step retires the copies made so far and leaves b live. ctx is not nil.
func infillWithSkin(ctx context.Context, b *Body, spec infillSpec) (result *Body, retErr error) {
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	// A step may retire only a body it built, so a refusal comes before any copy is made.
	if _, err := documentHandle(b.n.Doc).stepGate(ctx, nil, []*Body{b}); err != nil {
		return nil, err
	}
	var temporaries []*Body
	defer func() {
		if retErr != nil {
			removeTools(ctx, temporaries)
		}
	}()
	skinSource, err := copyAsIs(ctx, b)
	if err != nil {
		return nil, err
	}
	temporaries = append(temporaries, skinSource)
	fillSource, err := copyAsIs(ctx, b)
	if err != nil {
		return nil, err
	}
	temporaries = append(temporaries, fillSource)
	skin, err := shell(ctx, skinSource, nil, spec.skin, modifyoption.ShellConfig{NoOpenings: true}, nil)
	if err != nil {
		return nil, subjectFault(OperationInfill, spec.skinSubject, fmt.Errorf("decad: the infill skin: %w", err))
	}
	temporaries = append(temporaries, skin)
	interior := spec
	interior.skin, interior.skinSet = units.Value{}, false
	fill, err := infillInterior(ctx, fillSource, interior)
	if err != nil {
		return nil, err
	}
	temporaries = append(temporaries, fill)
	joined, err := union(ctx, []*Body{skin, fill})
	if err != nil {
		return nil, err
	}
	if err := removeConsumed(ctx, b); err != nil {
		return nil, err
	}
	return joined, nil
}

// infillInterior fills the whole of b with the lattice (Infill without a skin).
// ctx is not nil.
func infillInterior(ctx context.Context, b *Body, spec infillSpec) (*Body, error) {
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: Infill needs a solid body`, ErrNotSolid)
	}
	cell, err := spec.cell.In(units.Millimeter)
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
		Cell: spec.cell, Level: spec.level, Resolution: spec.resolution,
	}
	var build func(context.Context, LatticeSpec) (*MeshBody, error)
	switch spec.kind {
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
	case SplitPInfill:
		build = SplitPLattice
	default:
		return nil, fmt.Errorf(`%w: unknown infill lattice %s`, ErrDegenerate, spec.kind)
	}
	mesh, err := build(ctx, ls)
	if err != nil {
		if errors.Is(err, lattice.ErrResolution) {
			return nil, subjectFault(OperationInfill, spec.resolutionSubject, err)
		}
		return nil, err
	}
	lattice, err := solidFromMesh(ctx, documentHandle(b.n.Doc), mesh)
	if err != nil {
		// A cancel inside the conversion is the caller's, not a refusal of the lattice
		// (agent-docs/command-api-design.md §3).
		if cerr := ctx.Err(); cerr != nil {
			return nil, cerr
		}
		return nil, fmt.Errorf("%w: the infill lattice is not a solid the evaluator can prove: %s", ErrUnsupported, err)
	}
	return intersect(ctx, b, lattice, booleanConfig{})
}
