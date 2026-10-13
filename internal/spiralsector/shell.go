// Package spiralsector checks the spiral first-join and inverse-copy gates.
// Bounded shell separation is delegated to shellseparation.
package spiralsector

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/r3"
)

// Shell keeps the existing spiral caller shape while using the neutral proof.
type Shell = shellseparation.Shell

// HeldCopy keeps the spiral caller's provenance check at its source gate.
type HeldCopy = shellseparation.HeldCopy

func Separate(ctx context.Context, shells []Shell, moves []r3.Transform, held bool) bool {
	kind := shellseparation.SourceShell
	if held {
		kind = shellseparation.HeldCopyShell
	}
	return shellseparation.Separate(ctx, shells, moves, kind, shellseparation.AxisX, true)
}

func PlacedHeldShells(ctx context.Context, copies []HeldCopy, moves []r3.Transform) ([]Shell, error) {
	return shellseparation.PlacedHeldShells(ctx, copies, moves)
}

func ratZero() *big.Rat { return new(big.Rat) }

func ratOne() *big.Rat { return big.NewRat(1, 1) }

func ratF(v float64) *big.Rat { return proof.FloatRat(v) }

func ratAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }

func ratSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }

func ratMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }

func ratAbs(a *big.Rat) *big.Rat { return new(big.Rat).Abs(a) }
