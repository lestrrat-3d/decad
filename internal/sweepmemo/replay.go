package sweepmemo

import (
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// ReplayPoseCap bounds how many fractions one sweep's replay memo keeps.
const ReplayPoseCap = 32

// ReplayPoses is one replay answer: both certified poses, or the refusal.
type ReplayPoses struct {
	A, B r3.Transform
	Err  error
}

// ReplayFractionKey encodes a fraction's sign, numerator and denominator.
// big.Rat keeps lowest terms, so equal fractions share one key.
func ReplayFractionKey(f *big.Rat) string {
	num, den := f.Num().Bytes(), f.Denom().Bytes()
	key := make([]byte, 0, 5+len(num)+len(den))
	key = append(key, byte(f.Sign()+1), byte(len(num)>>24), byte(len(num)>>16), byte(len(num)>>8), byte(len(num)))
	key = append(key, num...)
	return string(append(key, den...))
}
