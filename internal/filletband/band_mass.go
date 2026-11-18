package filletband

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// BandMassInput names one fillet band's strip and its reference-frame map.
type BandMassInput struct {
	Pieces              []Piece
	Walks               []survey2d.SideWalk
	Corners             []Corner
	Rho                 []float64
	Restate             RestateTerms
	Radius              proofbound.RatInterval
	Level, LevelDelta   float64
	MaterialSign, Sigma float64
	Embed               brepgeom.Embed
}

// BandMass holds the band's contribution to three times volume and to each
// first moment in the record's reference frame.
type BandMass struct {
	Volume3 proofbound.RatInterval
	Moments [3]proofbound.RatInterval
}

// RadiusInterval encloses a held radius and its unit-conversion bound.
func RadiusInterval(held, delta float64) (proofbound.RatInterval, error) {
	r, d := proofarith.FloatRat(held), proofarith.FloatRat(delta)
	if r == nil || d == nil {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: a fillet radius is not finite`, decaderr.ErrNotFinite)
	}
	return proofbound.IntervalWiden(proofbound.PointInterval(r), d), nil
}

// MassOf integrates the fillet strip and widens every changed term at curved
// miters, at LF5′ corners (Rho, agent-docs/modify-design.md Part L §5.5), and
// for the loop's restated off-radius arcs (Restate, §5.6). The
// sign of either corner's change is unknown, so its allowance widens both
// ends of each interval.
func MassOf(in BandMassInput) (BandMass, error) {
	h, err := HeightsOf(in.Radius)
	if err != nil {
		return BandMass{}, err
	}
	level, delta := proofarith.FloatRat(in.Level), proofarith.FloatRat(in.LevelDelta)
	if level == nil || delta == nil {
		return BandMass{}, fmt.Errorf(`%w: a fillet band's face level is not finite`, decaderr.ErrNotFinite)
	}
	m := int64(in.MaterialSign)
	side := proofbound.IntervalWiden(proofbound.IntervalAdd(proofbound.PointInterval(level),
		proofbound.IntervalScale(in.Radius, big.NewRat(m, 1))), delta)
	strip := StripOf(SumCoefficients(in.Pieces), h, side, m)
	sigma := big.NewRat(int64(in.Sigma), 1)
	zero := proofbound.PointInterval(new(big.Rat))
	out := BandMass{Volume3: proofbound.IntervalScale(strip.Volume, new(big.Rat).Mul(sigma, big.NewRat(3, 1))),
		Moments: [3]proofbound.RatInterval{zero, zero, zero}}
	for i := range 3 {
		out.Moments[in.Embed.Axis[i]] = proofbound.IntervalScale(strip.Moment[i],
			new(big.Rat).Mul(sigma, big.NewRat(int64(in.Embed.Sign[i]), 1)))
	}
	if len(in.Corners) == 0 {
		return out, nil
	}
	zReach := new(big.Rat).Add(new(big.Rat).Abs(level), delta)
	zReach.Add(zReach, in.Radius.Hi)
	if in.Restate.Arcs > 0 {
		volume, moment, ok := RestateMassAllowance(in.Restate, WalkCoordUpper(in.Walks, 0), in.Radius)
		if !ok {
			return BandMass{}, fmt.Errorf(`%w: a fillet's restated arcs state no mass allowance`, decaderr.ErrUnsupported)
		}
		out.Volume3 = proofbound.IntervalWiden(out.Volume3, new(big.Rat).Mul(volume, big.NewRat(3, 1)))
		out.Moments[in.Embed.Axis[0]] = proofbound.IntervalWiden(out.Moments[in.Embed.Axis[0]], moment)
		out.Moments[in.Embed.Axis[1]] = proofbound.IntervalWiden(out.Moments[in.Embed.Axis[1]], moment)
		out.Moments[in.Embed.Axis[2]] = proofbound.IntervalWiden(out.Moments[in.Embed.Axis[2]], new(big.Rat).Mul(volume, zReach))
	}
	for k, class := range in.Corners {
		var volume, mu, mv *big.Rat
		var ok bool
		switch class {
		case CurvedMiter:
			prev := in.Walks[(k+len(in.Walks)-1)%len(in.Walks)]
			volume, mu, mv, ok = CurvedMiterMassAllowance(prev, in.Walks[k], in.Radius)
			if !ok {
				return BandMass{}, fmt.Errorf(`%w: a curved fillet's strip has no mass allowance`, decaderr.ErrUnsupported)
			}
		case NearTangent:
			if k >= len(in.Rho) {
				return BandMass{}, fmt.Errorf(`%w: a near-tangent fillet corner has no reach`, decaderr.ErrUnsupported)
			}
			w := in.Walks[k]
			volume, mu, mv, ok = NearTangentMassAllowance(w.StartU, w.StartV,
				proofbound.WalkEndBoundAllow(w.StartBound), in.Rho[k], in.Radius)
			if !ok {
				return BandMass{}, fmt.Errorf(`%w: a near-tangent fillet's strip has no mass allowance`, decaderr.ErrUnsupported)
			}
		default:
			continue
		}
		out.Volume3 = proofbound.IntervalWiden(out.Volume3, new(big.Rat).Mul(volume, big.NewRat(3, 1)))
		out.Moments[in.Embed.Axis[0]] = proofbound.IntervalWiden(out.Moments[in.Embed.Axis[0]], mu)
		out.Moments[in.Embed.Axis[1]] = proofbound.IntervalWiden(out.Moments[in.Embed.Axis[1]], mv)
		out.Moments[in.Embed.Axis[2]] = proofbound.IntervalWiden(out.Moments[in.Embed.Axis[2]],
			new(big.Rat).Mul(volume, zReach))
	}
	return out, nil
}
