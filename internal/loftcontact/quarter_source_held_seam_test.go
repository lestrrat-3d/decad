package loftcontact

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func quarterTestHeldPiece(x *big.Rat) heldPiece {
	return heldPiece{
		lo: heldPoint{x: x, z: big.NewRat(0, 1)},
		hi: heldPoint{x: x, z: big.NewRat(10, 1)},
	}
}

func TestQuarterSourceHeldSeparationRefusals(t *testing.T) {
	m := loftHullMath{budget: proofbound.NewWallWorkBudget(65_536)}
	outer := []heldPiece{quarterTestHeldPiece(big.NewRat(4, 1))}
	inner := []heldPiece{quarterTestHeldPiece(big.NewRat(7, 2))}
	slabs, separated, err := quarterSourceHeldSeparation(t.Context(), m,
		outer, inner, big.NewRat(1, 4))
	if err != nil || separated || slabs != 0 {
		t.Fatalf("touching bands: separated=%t slabs=%d err=%v", separated, slabs, err)
	}

	large := new(big.Int).Lsh(big.NewInt(1), 8191)
	outer = []heldPiece{quarterTestHeldPiece(new(big.Rat).SetInt(large))}
	slabs, separated, err = quarterSourceHeldSeparation(t.Context(), m,
		outer, inner, big.NewRat(1, 4))
	if separated || slabs != 0 || !errors.Is(err, errLoftHullSize) {
		t.Fatalf("oversized arithmetic: separated=%t slabs=%d err=%v", separated, slabs, err)
	}
	report, accepted, err := CertifyQuarterSourceHeldSelected(t.Context(),
		proofbound.NewWallWorkBudget(65_536), nil, nil, 0,
		[2]*big.Rat{new(big.Rat).SetInt(large), big.NewRat(11, 2)}, 3)
	if accepted || err != nil || report.Reason != "arithmetic_size" ||
		report.Y1.Bound != nil || report.Y3.Bound != nil {
		t.Fatalf("oversized radius: accepted=%t reason=%s err=%v", accepted, report.Reason, err)
	}
}
