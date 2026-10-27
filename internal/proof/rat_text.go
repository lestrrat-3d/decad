package proof

import "math/big"

// RatTextEqual reports whether a.RatString() == b.RatString() without
// building either text. RatString is the numerator's decimal text, followed
// by "/" and the denominator's text unless IsInt holds, and Int.String is
// injective. So the texts agree exactly when IsInt agrees, the numerators are
// equal, and, for a non-integer, the denominators are equal. Neither rational
// need be in lowest terms.
func RatTextEqual(a, b *big.Rat) bool {
	ai := a.IsInt()
	if ai != b.IsInt() || a.Num().Cmp(b.Num()) != 0 {
		return false
	}
	return ai || a.Denom().Cmp(b.Denom()) == 0
}

// RatTextHash folds into h the integers RatTextEqual compares, so two
// rationals it calls equal fold equally. Denom is read only when IsInt fails,
// where it is a reference to the stored denominator and allocates nothing.
func RatTextHash(h uint64, r *big.Rat) uint64 {
	h = IntHash(h, r.Num())
	if r.IsInt() {
		return HashWord(h, 0)
	}
	return IntHash(HashWord(h, 1), r.Denom())
}

// HashSeed starts a HashWord chain.
const HashSeed = 14695981039346656037

// HashWord folds one word into h (FNV-1a over whole words). It is a lookup
// hash only: callers confirm every hit by exact comparison.
func HashWord(h, w uint64) uint64 { return (h ^ w) * 1099511628211 }

// IntHash folds x's sign, word count and magnitude words into h.
func IntHash(h uint64, x *big.Int) uint64 {
	words := x.Bits()
	h = HashWord(h, uint64(x.Sign()+1)<<32|uint64(len(words)))
	for _, w := range words {
		h = HashWord(h, uint64(w))
	}
	return h
}
