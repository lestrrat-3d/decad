package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/lattice"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// defaultLatticeResolution is the samples to a cell a [LatticeSpec] with no
// Resolution takes.
const defaultLatticeResolution = 16

// LatticeSpec describes a lattice for [GyroidLattice], [SchwarzPLattice],
// [DiamondLattice], [NeoviusLattice], [IWPLattice], [FischerKochSLattice], [LidinoidLattice] and [SplitPLattice].
type LatticeSpec struct {
	// Min and Max are the box's corners in millimetres; Min is below Max on every
	// axis.
	Min, Max r3.Vec
	// Cell is the length after which the lattice repeats along each axis, a
	// positive length.
	Cell units.Value
	// Level is a dimensionless wall thickness: the solid is where the lattice's
	// field g lies within Level of zero, |g| ≤ Level. The gyroid's g runs from
	// −3/2 to 3/2, so a level of 0.3 gives a thin wall, 1 a heavy one and values
	// near 3/2 a nearly full box; the Schwarz-P field runs from −3 to 3 and the
	// diamond's from −√2 to √2, the Neovius field from −13 to 13, the IWP field
	// from −5 to 3, the Fischer-Koch S field from −√2 to √2 and the Lidinoid from −1.35 to 1.15 and the Split P from −1.8 to about 1.81. Level lies strictly between 0 and the field's largest absolute
	// value (agent-docs/lattice-design.md §4).
	Level units.Value
	// Resolution is the number of samples to a cell along each axis, from 4 up.
	// The zero value takes 16. A finer grid follows the surface more closely and
	// makes more triangles.
	Resolution int
}

// GyroidLattice builds a gyroid lattice, the triply periodic wall used for
// lightweight print infill, as a closed [MeshBody] filling the box of spec
// (agent-docs/lattice-design.md). The box's faces cap the lattice, so the mesh
// is closed and wound outward, and its [MeshBody.Volume] reads.
//
// The surface is the zero set of the field sampled on a grid and cut by
// marching tetrahedra, so it is as exact as the grid is fine: the mesh is not a
// B-rep, and its volume converges on the lattice's as Resolution grows. The
// lattice fills a box, not a body: nothing here takes a mesh body into a Boolean.
//
// ctx MUST NOT be nil. A nil context, a box with no volume, a cell that is not
// positive, a level not between 0 and 3/2 (the gyroid field's largest value), a Resolution below 4, and a lattice
// that needs more than 8 million samples or 2 million triangles are
// ErrDegenerate. A cell of the wrong unit kind is ErrUnitKind and a negative one
// ErrNegativeMagnitude. A level that is not dimensionless is ErrUnitKind.
// Non-finite corners are ErrNotFinite. A canceled context returns its error.
func GyroidLattice(ctx context.Context, spec LatticeSpec) (*MeshBody, error) {
	return buildLattice(ctx, spec, lattice.KindGyroid)
}

// SchwarzPLattice builds a Schwarz primitive (P) lattice, cos kx + cos ky + cos kz,
// a triply periodic wall of cubic symmetry whose solid is where |g| ≤ Level, as a
// closed [MeshBody] filling the box of spec (agent-docs/lattice-design.md §4).
// Everything GyroidLattice says holds, except that the field runs from −3 to 3 and
// Level lies strictly between 0 and 3.
func SchwarzPLattice(ctx context.Context, spec LatticeSpec) (*MeshBody, error) {
	return buildLattice(ctx, spec, lattice.KindSchwarzP)
}

// DiamondLattice builds a Schwarz diamond lattice, the field
// sin kx·sin ky·sin kz + sin kx·cos ky·cos kz + cos kx·sin ky·cos kz +
// cos kx·cos ky·sin kz, whose solid is where |g| ≤ Level, as a closed
// [MeshBody] filling the box of spec (agent-docs/lattice-design.md §4).
// Everything GyroidLattice says holds, except that the field runs from −√2 to √2
// and Level lies strictly between 0 and √2.
func DiamondLattice(ctx context.Context, spec LatticeSpec) (*MeshBody, error) {
	return buildLattice(ctx, spec, lattice.KindDiamond)
}

// NeoviusLattice builds a Neovius lattice, the field
// 3(cos kx + cos ky + cos kz) + 4 cos kx·cos ky·cos kz, whose solid is where
// |g| ≤ Level, as a closed [MeshBody] filling the box of spec
// (agent-docs/lattice-design.md §4). Everything GyroidLattice says holds, except
// that the field runs from −13 to 13 and Level lies strictly between 0 and 13.
func NeoviusLattice(ctx context.Context, spec LatticeSpec) (*MeshBody, error) {
	return buildLattice(ctx, spec, lattice.KindNeovius)
}

// FischerKochSLattice builds a Fischer-Koch S lattice, the field
// cos 2kx·sin ky·cos kz + cos kx·cos 2ky·sin kz + sin kx·cos ky·cos 2kz, whose solid is where
// |g| ≤ Level, as a closed [MeshBody] filling the box of spec (agent-docs/lattice-design.md §4). Everything
// GyroidLattice says holds, except that the field runs from −√2 to √2 and Level lies strictly between 0 and √2.
func FischerKochSLattice(ctx context.Context, spec LatticeSpec) (*MeshBody, error) {
	return buildLattice(ctx, spec, lattice.KindFischerKochS)
}

// LidinoidLattice builds a Lidinoid lattice, the field
// ½(sin 2kx·cos ky·sin kz + sin 2ky·cos kz·sin kx + sin 2kz·cos kx·sin ky) −
// ½(cos 2kx·cos 2ky + cos 2ky·cos 2kz + cos 2kz·cos 2kx) + 0.15, whose solid is where |g| ≤ Level, as a closed
// [MeshBody] filling the box of spec (agent-docs/lattice-design.md §4). Everything GyroidLattice says holds, except
// that the field runs from −1.35 to 1.15, so Level lies strictly between 0 and 1.35, the largest |g|.
func LidinoidLattice(ctx context.Context, spec LatticeSpec) (*MeshBody, error) {
	return buildLattice(ctx, spec, lattice.KindLidinoid)
}

// SplitPLattice builds a Split P lattice, the field
// 1.1(sin 2kx·sin kz·cos ky + sin 2ky·sin kx·cos kz + sin 2kz·sin ky·cos kx) −
// 0.2(cos 2kx·cos 2ky + cos 2ky·cos 2kz + cos 2kz·cos 2kx) − 0.4(cos 2kx + cos 2ky + cos 2kz), whose solid is where
// |g| ≤ Level, as a closed [MeshBody] filling the box of spec (agent-docs/lattice-design.md §4). Everything
// GyroidLattice says holds, except that the field runs from −1.8 to about 1.81 and Level lies strictly between 0 and
// 1.8.
func SplitPLattice(ctx context.Context, spec LatticeSpec) (*MeshBody, error) {
	return buildLattice(ctx, spec, lattice.KindSplitP)
}

// IWPLattice builds an I-graph and wrapped package (IWP) lattice, the field
// 2(cos kx·cos ky + cos ky·cos kz + cos kz·cos kx) − (cos 2kx + cos 2ky + cos 2kz),
// whose solid is where |g| ≤ Level, as a closed [MeshBody] filling the box of
// spec (agent-docs/lattice-design.md §4). Everything GyroidLattice says holds,
// except that the field runs from −5 to 3, so Level lies strictly between 0 and
// 5, the largest |g|.
func IWPLattice(ctx context.Context, spec LatticeSpec) (*MeshBody, error) {
	return buildLattice(ctx, spec, lattice.KindIWP)
}

// buildLattice is the gates and the build shared by the lattice kinds.
func buildLattice(ctx context.Context, spec LatticeSpec, kind lattice.Kind) (*MeshBody, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a lattice`, ErrDegenerate)
	}
	for _, c := range []float64{spec.Min.X, spec.Min.Y, spec.Min.Z, spec.Max.X, spec.Max.Y, spec.Max.Z} {
		if math.IsNaN(c) || math.IsInf(c, 0) {
			return nil, fmt.Errorf(`%w: a lattice corner is not finite`, ErrNotFinite)
		}
	}
	cell, err := sectionrecord.MagnitudeIn(spec.Cell, units.Length, units.Millimeter, "the lattice cell")
	if err != nil {
		return nil, err
	}
	if spec.Level.Kind() != units.Dimensionless {
		return nil, fmt.Errorf(`%w: the lattice level must be dimensionless`, ErrUnitKind)
	}
	level := spec.Level.Base()
	if math.IsNaN(level) || math.IsInf(level, 0) {
		return nil, fmt.Errorf(`%w: the lattice level is not finite`, ErrNotFinite)
	}
	res := spec.Resolution
	if res == 0 {
		res = defaultLatticeResolution
	}
	vertices, triangles, err := lattice.Surface(ctx, kind, spec.Min, spec.Max, cell, level, res)
	if errors.Is(err, lattice.ErrDegenerate) {
		return nil, fmt.Errorf("%w: %s", ErrDegenerate, err)
	}
	if err != nil {
		return nil, err
	}
	// Surface returns fresh slices that buildLattice does not retain or mutate.
	// This transfers their ownership to the mesh body.
	return newMeshBody(ctx, vertices, triangles, false)
}
