package proof

import (
	"fmt"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These refusals are constructed once so rejected replay does not allocate.
var (
	errNormalReplay1 = fmt.Errorf("%w: exact normal storage overflow", decaderr.ErrUnsupported)
)

func normalReserveLimbs(reserve func(uint64) error, limbs, objects uint64) error {
	if limbs > (^uint64(0)-63)/uint64(bits.UintSize/8) {
		return errNormalReplay1
	}
	blocks := (limbs*uint64(bits.UintSize/8) + 63) / 64
	if objects > ^uint64(0)-blocks {
		return errNormalReplay1
	}
	return reserve(blocks + objects)
}

func normalChargeDifference(a, b Dyadic, reserve, spend func(uint64) error) error {
	base, top := 0, 0
	if !a.IsZero() {
		base = a.exp
		top = a.exp + a.bitLen()
	}
	if !b.IsZero() {
		if a.IsZero() {
			base = b.exp
			top = b.exp + b.bitLen()
		} else {
			base = min(base, b.exp)
			top = max(top, b.exp+b.bitLen())
		}
	}
	width := max(1, top-base+1)
	words := uint64((width + bits.UintSize - 1) / bits.UintSize)
	// Two aligned inputs, output, normalized result and spare limbs for
	// math/big's add/subtract operations; no multiplication is used here.
	if err := normalReserveLimbs(reserve, 6*(words+4), 6); err != nil {
		return err
	}
	return spend(12*words + 6)
}

func normalMulReplay(a, b Dyadic, reserve, spend func(uint64) error) (Dyadic, error) {
	if a.IsZero() || b.IsZero() {
		if err := spend(1); err != nil {
			return Dyadic{}, err
		}
		return Dyadic{}, nil
	}
	aw := uint64((a.bitLen() + bits.UintSize - 1) / bits.UintSize)
	bw := uint64((b.bitLen() + bits.UintSize - 1) / bits.UintSize)
	// Two input views and one exact output array. The explicit kernel has no
	// recursive multiplication or hidden scratch allocation.
	if err := normalReserveLimbs(reserve, 2*(aw+bw)+8, 4); err != nil {
		return Dyadic{}, err
	}
	if err := spend(aw*bw + 3*(aw+bw) + 1); err != nil {
		return Dyadic{}, err
	}
	var ai, bi big.Int
	var ab, bb dyBuf
	ax, bx := a.view(&ai, &ab).Bits(), b.view(&bi, &bb).Bits()
	out := make([]big.Word, len(ax)+len(bx))
	for i, x := range ax {
		carry := uint(0)
		for j, y := range bx {
			hi, lo := bits.Mul(uint(x), uint(y))
			lo, c := bits.Add(lo, uint(out[i+j]), 0)
			hi, _ = bits.Add(hi, 0, c)
			lo, c = bits.Add(lo, carry, 0)
			hi, _ = bits.Add(hi, 0, c)
			out[i+j] = big.Word(lo)
			carry = hi
		}
		out[i+len(bx)] = big.Word(carry)
	}
	z := new(big.Int).SetBits(out)
	if (a.Sign() < 0) != (b.Sign() < 0) {
		z.Neg(z)
	}
	return fromBig(z, a.exp+b.exp), nil
}
