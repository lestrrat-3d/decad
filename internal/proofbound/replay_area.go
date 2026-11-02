package proofbound

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proof "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

type replayAreaMeter struct{ b *WorkBudget }

func (m replayAreaMeter) ReserveRecords(_ context.Context, n uint64) error { return m.b.Reserve(n) }
func (m replayAreaMeter) ReserveBytes(_ context.Context, n uint64) error   { return m.b.ReserveBytes(n) }
func (m replayAreaMeter) SpendWork(_ context.Context, n uint64) error {
	return m.b.SpendWork(n)
}
func replayAreaTerms(ctx context.Context, b *WorkBudget, verts []r3.Vec, tris [][3]int) ([]perturbedTerm, error) {
	meter := replayAreaMeter{b}
	if err := b.Reserve(uint64(len(tris))); err != nil {
		return nil, err
	}
	if err := b.ReserveBytesMul(uint64(len(tris)), uint64(unsafe.Sizeof(perturbedTerm{}))); err != nil {
		return nil, err
	}
	terms := make([]perturbedTerm, len(tris))
	norm := func(v [3]*big.Rat, quarter bool) (float64, error) {
		if err := b.ReserveBytesMul(1, uint64(unsafe.Sizeof(big.Rat{}))); err != nil {
			return 0, err
		}
		sum := new(big.Rat)
		for _, x := range v {
			sq, err := proof.ReplayRatArithmetic(ctx, x, x, '*', meter)
			if err != nil {
				return 0, err
			}
			sum, err = proof.ReplayRatArithmetic(ctx, sum, sq, '+', meter)
			if err != nil {
				return 0, err
			}
		}
		if quarter {
			if err := b.ReserveBytesMul(1, 256); err != nil {
				return 0, err
			}
			four := big.NewRat(4, 1)
			var err error
			sum, err = proof.ReplayRatArithmetic(ctx, sum, four, '/', meter)
			if err != nil {
				return 0, err
			}
		}
		return proof.ReplaySqrtDyadicRat(ctx, sum, meter)
	}
	for i, t := range tris {
		if err := b.Step(); err != nil {
			return nil, err
		}
		for _, v := range t {
			if v < 0 || v >= len(verts) {
				return nil, fmt.Errorf("%w: replay area facet index", decaderr.ErrUnsupported)
			}
		}
		a, c, d := verts[t[0]], verts[t[1]], verts[t[2]]
		if !FiniteVec(a) || !FiniteVec(c) || !FiniteVec(d) {
			continue
		}
		var u, v [3]*big.Rat
		for j := range 3 {
			aa := [3]float64{a.X, a.Y, a.Z}[j]
			cc := [3]float64{c.X, c.Y, c.Z}[j]
			dd := [3]float64{d.X, d.Y, d.Z}[j]
			ar, err := proof.ReplayFloatRat(ctx, aa, meter)
			if err != nil {
				return nil, err
			}
			cr, err := proof.ReplayFloatRat(ctx, cc, meter)
			if err != nil {
				return nil, err
			}
			dr, err := proof.ReplayFloatRat(ctx, dd, meter)
			if err != nil {
				return nil, err
			}
			u[j], err = proof.ReplayRatArithmetic(ctx, cr, ar, '-', meter)
			if err != nil {
				return nil, err
			}
			v[j], err = proof.ReplayRatArithmetic(ctx, dr, ar, '-', meter)
			if err != nil {
				return nil, err
			}
		}
		var cross [3]*big.Rat
		for j := range 3 {
			x, y := (j+1)%3, (j+2)%3
			l, err := proof.ReplayRatArithmetic(ctx, u[x], v[y], '*', meter)
			if err != nil {
				return nil, err
			}
			r, err := proof.ReplayRatArithmetic(ctx, u[y], v[x], '*', meter)
			if err != nil {
				return nil, err
			}
			cross[j], err = proof.ReplayRatArithmetic(ctx, l, r, '-', meter)
			if err != nil {
				return nil, err
			}
		}
		area, err := norm(cross, true)
		if err != nil {
			return nil, err
		}
		lu, err := norm(u, false)
		if err != nil {
			return nil, err
		}
		lv, err := norm(v, false)
		if err != nil {
			return nil, err
		}
		terms[i] = perturbedTerm{area: area, lu: lu, lv: lv, ok: true}
	}
	return terms, nil
}

// PerturbedAreaUpperReplay performs the serial area proof with the caller context.
func PerturbedAreaUpperReplay(ctx context.Context, b *WorkBudget, verts []r3.Vec, tris [][3]int, delta float64) (float64, error) {
	if b == nil || !b.Replay {
		return 0, fmt.Errorf("%w: missing replay area budget", decaderr.ErrUnsupported)
	}
	terms, err := replayAreaTerms(ctx, b, verts, tris)
	if err != nil {
		return 0, err
	}
	total := 0.0
	for _, term := range terms {
		if !term.ok {
			total += math.Inf(1)
			continue
		}
		total += AbsSumUpper(term.area, perturbedAllow(delta, term.lu, term.lv))
	}
	if err := b.Err(); err != nil {
		return 0, err
	}
	return UpRound(total + SumSlop(len(tris), total)), nil
}

// PerturbedAreaUpperAllowsReplay also returns the per-facet allowance.
func PerturbedAreaUpperAllowsReplay(ctx context.Context, b *WorkBudget, verts []r3.Vec, tris [][3]int, delta float64) (float64, []float64, error) {
	if b == nil || !b.Replay {
		return 0, nil, fmt.Errorf("%w: missing replay area budget", decaderr.ErrUnsupported)
	}
	terms, err := replayAreaTerms(ctx, b, verts, tris)
	if err != nil {
		return 0, nil, err
	}
	if err := b.ReserveBytesMul(uint64(len(tris)), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return 0, nil, err
	}
	allows := make([]float64, len(tris))
	total := 0.0
	for i, term := range terms {
		if !term.ok {
			allows[i] = math.Inf(1)
			total += math.Inf(1)
			continue
		}
		allows[i] = perturbedAllow(delta, term.lu, term.lv)
		total += AbsSumUpper(term.area, allows[i])
	}
	if err := b.Err(); err != nil {
		return 0, nil, err
	}
	return UpRound(total + SumSlop(len(tris), total)), allows, nil
}
