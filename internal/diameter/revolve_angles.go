package diameter

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
)

// GateAngle is a held sweep angle with enclosures of the sine and cosine of
// the angle it denotes.
type GateAngle struct {
	Phi      float64
	Sin, Cos proofbound.RatInterval
}

// RevolveGateAngles reads both sweep ends and a half-turn station when its
// denotation is proven inside the sweep. The half-turn station denotes exact
// quarter turns when the start does; otherwise it denotes its held angle.
// An end without an angle returns false.
func RevolveGateAngles(phi0, phi1 float64, den0, den1 revolveangle.Angle) ([]GateAngle, bool) {
	at := func(phi float64, den revolveangle.Angle) (GateAngle, bool) {
		if !den.Valid() {
			return GateAngle{}, false
		}
		sin, cos, ok := den.SinCosFor(phi)
		return GateAngle{Phi: phi, Sin: sin, Cos: cos}, ok
	}
	start, ok0 := at(phi0, den0)
	end, ok1 := at(phi1, den1)
	if !ok0 || !ok1 {
		return nil, false
	}
	angles := []GateAngle{start, end}
	phi := phi0 + math.Pi
	den := revolveangle.Angle{Rad: proofarith.FloatRat(phi), Turn: new(big.Rat)}
	if den0.Span == nil && den0.Rad.Sign() == 0 {
		den = revolveangle.Angle{Rad: new(big.Rat), Turn: new(big.Rat).Add(den0.Turn, big.NewRat(1, 2))}
	}
	enc, ok := den.Enclosure()
	enc0, ok0 := den0.Enclosure()
	enc1, ok1 := den1.Enclosure()
	if !ok || !ok0 || !ok1 || enc.Lo.Cmp(enc0.Hi) < 0 || enc.Hi.Cmp(enc1.Lo) > 0 {
		return angles, true
	}
	if half, ok := at(phi, den); ok {
		angles = append(angles, half)
	}
	return angles, true
}
