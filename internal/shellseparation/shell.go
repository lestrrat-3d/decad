// Package shellseparation proves bounded separation of recorded source and
// held-copy shells. Source provenance is checked by the caller before passing
// shells; this package checks their complete geometric bounds and placement.
package shellseparation

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// Axis selects the longitudinal coordinate used for closed slab proofs.
// The other two Cartesian coordinates form the projection plane.
type Axis uint8

const (
	AxisX Axis = iota
	AxisY
	AxisZ
)

// Kind states which bound each shell carries. Source bounds enclose true
// geometry; held-copy and proxy bounds enclose their separate held images.
type Kind uint8

const (
	SourceShell Kind = iota
	HeldCopyShell
	ProxyShell
)

// Shell is one audited, closed source or held shell and its two-sided bounds.
// The caller audits closure and orientation before calling Separate. Each
// vertex has one finite, nonnegative bound; MeshBound encloses all of them.
type Shell struct {
	Verts       []r3.Vec
	Tris        [][3]int
	VertexBound []float64
	MeshBound   float64
}

func ratOne() *big.Rat { return big.NewRat(1, 1) }

func ratZero() *big.Rat { return new(big.Rat) }

func ratF(v float64) *big.Rat { return proof.FloatRat(v) }

func ratAdd(a, b *big.Rat) *big.Rat { return proof.AddRat(new(big.Rat), a, b) }

func ratSub(a, b *big.Rat) *big.Rat { return proof.SubRat(new(big.Rat), a, b) }

func ratMul(a, b *big.Rat) *big.Rat { return proof.MulRat(new(big.Rat), a, b) }

func ratAbs(a *big.Rat) *big.Rat { return new(big.Rat).Abs(a) }
