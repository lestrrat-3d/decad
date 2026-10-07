package motionbound

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// DefaultResolution reports |To − From|/1024 in From's unit. If it
// underflows, it reports the smallest positive value whose base conversion
// is nonzero.
func DefaultResolution(kind units.Kind, from, to units.Value, fromP, toP MotionParam) (units.Value, bool) {
	var d *big.Rat
	if kind == units.Length {
		// The exact base-unit difference survives conversion that could round
		// two distinct endpoints to the same float in From's unit.
		d = new(big.Rat).Sub(toP.Base, fromP.Base)
		d.Abs(d)
		d.Quo(d, proofarith.FloatRat(from.Unit().Factor()))
	} else {
		fromMag := proofarith.FloatRat(from.Mag())
		toMag, err := to.In(from.Unit())
		toRat := proofarith.FloatRat(toMag)
		if err != nil || fromMag == nil || toRat == nil {
			return units.New(0, from.Unit()), false
		}
		d = new(big.Rat).Sub(toRat, fromMag)
		d.Abs(d)
	}
	mag, _ := d.Quo(d, big.NewRat(1024, 1)).Float64()
	base, _ := units.BaseUnit(kind)
	reported := units.New(mag, from.Unit())
	converted, err := reported.In(base)
	if mag > 0 && err == nil && converted > 0 {
		return reported, false
	}
	// A positive exact floor can underflow in From's unit or its base unit.
	// Binary search positive finite float bits for the first value whose base
	// conversion is nonzero, which is also the first WithResolution accepts.
	low, high := uint64(0), math.Float64bits(1)
	for high-low > 1 {
		mid := low + (high-low)/2
		candidate := units.New(math.Float64frombits(mid), from.Unit())
		converted, err := candidate.In(base)
		if err != nil || converted == 0 {
			low = mid
			continue
		}
		high = mid
	}
	return units.New(math.Float64frombits(high), from.Unit()), true
}

// DefaultResolutionParam returns one 1024th of each exact endpoint span.
func DefaultResolutionParam(fromP, toP MotionParam) MotionParam {
	part := func(a, b *big.Rat) *big.Rat {
		d := new(big.Rat).Sub(b, a)
		d.Abs(d)
		return d.Quo(d, big.NewRat(1024, 1))
	}
	return MotionParam{Turn: part(fromP.Turn, toP.Turn), Base: part(fromP.Base, toP.Base)}
}

// DomainLabel returns the stated endpoint at either end and the interpolated
// value in From's unit elsewhere.
func DomainLabel(from, to units.Value, f *big.Rat) units.Value {
	switch {
	case f.Sign() == 0:
		return from
	case f.Cmp(big.NewRat(1, 1)) == 0:
		return to
	}
	fromMag := proofarith.FloatRat(from.Mag())
	toMag, err := to.In(from.Unit())
	toRat := proofarith.FloatRat(toMag)
	if err != nil || fromMag == nil || toRat == nil {
		// Unreachable for a validated motion, whose endpoints both convert;
		// the exact parameter still governs every bound if it were reached.
		return from
	}
	d := new(big.Rat).Sub(toRat, fromMag)
	d.Mul(d, f)
	mag, _ := d.Add(d, fromMag).Float64()
	return units.New(mag, from.Unit())
}
