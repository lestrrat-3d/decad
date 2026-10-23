package revolveloftfaces

import (
	"math/big"
)

// A joint cell confines one source-side change to a one-sided Bezier sliver
// at an authenticated fit knot. The source polynomial is strictly monotone
// in that sliver for every time in the exact root bracket; the rest of the
// source span has one strict side. Its mapped X hull lies within the finite
// Revolve section. No held vertex enters this certificate.
type RevolveLoftFacesJointCell struct {
	SLo, SHi    *big.Rat
	Time        loftFaceInterval
	OutsideSide int
	SDerivative int
	X           loftFaceInterval
	Finite      bool
}
