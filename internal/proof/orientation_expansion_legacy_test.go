package proof

// Test-only copy of the OrientSignExpansion implementation that preceded the
// compensated-sum filter, kept as the differential reference: it builds the
// full grow-expansion for every admitted input.

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

func LegacyOrientSignExpansion(a, b, c, d r3.Vec) (int, bool) {
	ba, ca, da := b.Sub(a), c.Sub(a), d.Sub(a)
	for _, row := range [...][3]float64{
		{b.X, a.X, ba.X}, {b.Y, a.Y, ba.Y}, {b.Z, a.Z, ba.Z},
		{c.X, a.X, ca.X}, {c.Y, a.Y, ca.Y}, {c.Z, a.Z, ca.Z},
		{d.X, a.X, da.X}, {d.Y, a.Y, da.Y}, {d.Z, a.Z, da.Z},
	} {
		x := Abs(row[2])
		if !(x == 0 || x >= 0x1p-200 && x <= 0x1p200) {
			return 0, false
		}
		virtualB := row[0] - row[2]
		virtualA := row[2] + virtualB
		tail := (row[0] - virtualA) + (virtualB - row[1])
		if tail != 0 {
			return 0, false
		}
	}
	var sum legacyOrientationExpansion
	for _, term := range [...][3]float64{
		{ba.X, ca.Y, da.Z}, {ba.Y, ca.Z, da.X}, {ba.Z, ca.X, da.Y},
		{-ba.X, ca.Z, da.Y}, {-ba.Y, ca.X, da.Z}, {-ba.Z, ca.Y, da.X},
	} {
		xy := float64(term[0] * term[1])
		xyTail := math.FMA(term[0], term[1], -xy)
		hi := float64(xy * term[2])
		lo := math.FMA(xy, term[2], -hi)
		if xyTail != 0 {
			tailHi := float64(xyTail * term[2])
			tailLo := math.FMA(xyTail, term[2], -tailHi)
			sum.add(tailLo)
			sum.add(tailHi)
		}
		sum.add(lo)
		sum.add(hi)
	}
	if sum.n == 0 {
		return 0, true
	}
	if sum.terms[sum.n-1] > 0 {
		return 1, true
	}
	return -1, true
}

type legacyOrientationExpansion struct {
	terms [24]float64
	n     int
}

func (e *legacyOrientationExpansion) add(q float64) {
	if q == 0 {
		return
	}
	used := 0
	for _, component := range e.terms[:e.n] {
		sum := q + component
		virtual := sum - q
		low := (q - (sum - virtual)) + (component - virtual)
		if low != 0 {
			e.terms[used] = low
			used++
		}
		q = sum
	}
	if q != 0 {
		e.terms[used] = q
		used++
	}
	e.n = used
}

// OrientSignSum2 exposes the compensated-sum filter to the external tests,
// which check that each of its fixtures reaches the leg it is aimed at.
func OrientSignSum2(a, b, c, d r3.Vec) (int, bool) {
	return orientSignSum2(b.Sub(a), c.Sub(a), d.Sub(a))
}
