package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file reads cap-blend face tags and placed frames for the
// exact normal model in internal/capband/normal_model.go.

type capPatchModel = capband.NormalModel

// capPatchNormalModel adapts the published face and placed frame for capband.
func capPatchNormalModel(f *Face, pl prismPayload, g capPatchGeom, p r3.Vec) (capPatchModel, bool) {
	sinH, cosH, origin, axis, ok := coneTagTerms(f)
	if !ok {
		return capPatchModel{}, false
	}
	world, ok := newPlacedFrameMap(pl)
	if !ok {
		return capPatchModel{}, false
	}
	return capband.PatchNormalModel(capband.NormalInput{
		SinH: sinH, CosH: cosH, Origin: origin, Axis: axis, Pull: p,
		World: world, CU: g.cU, CV: g.cV, CapZ: g.capZ,
		Radius: g.capRadius, Th0: g.th0, Reversed: f.reversed,
	})
}

// coneTagTerms reads the sine and cosine of a circular patch's own half angle
// beside the axis they lean against. A `Cylinder` is the zero-half-angle member
// of the same family — `Face.NormalAt` hands back the bare radial direction
// there — so it takes the exact pair rather than a second code path. Any other
// tag refuses: a patch whose surface this file cannot state exactly gets no
// model at all, and DX7 answers undecided.
func coneTagTerms(f *Face) (proofbound.RatInterval, proofbound.RatInterval, r3.Vec, r3.Vec, bool) {
	switch s := f.surface.(type) {
	case Cone:
		half, err := s.HalfAngle.In(units.Radian)
		if err != nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		rHalf := proofarith.FloatRat(half)
		if rHalf == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		sin, cos, ok := proofbound.RadSinCosInterval(rHalf)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		return sin, cos, s.Origin, s.Axis, true
	case Cylinder:
		return proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(big.NewRat(1, 1)), s.Origin, s.Axis, true
	default:
		return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
	}
}

type harmonicExtremes struct {
	minLo, minHi, maxLo, maxHi *big.Rat
}

func harmonicWindowRange(a, b, c, width *big.Rat, wholeTurn bool) (harmonicExtremes, bool) {
	ext, ok := capband.HarmonicWindowRange(a, b, c, width, wholeTurn)
	if !ok {
		return harmonicExtremes{}, false
	}
	return harmonicExtremes{
		minLo: ext.MinLo, minHi: ext.MinHi, maxLo: ext.MaxLo, maxHi: ext.MaxHi,
	}, true
}

func newPlacedFrameMap(pp prismPayload) (survey2d.PlacedFrameMap, bool) {
	basis := pp.xform.Basis()
	ex, okX := proofbound.IvVec3Of(basis.EX)
	ey, okY := proofbound.IvVec3Of(basis.EY)
	ez, okZ := proofbound.IvVec3Of(basis.EZ)
	translation, okT := proofbound.IvVec3Of(pp.xform.Translation())
	origin, okO := proofbound.IvVec3Of(pp.frame.Origin())
	u, okU := proofbound.IvVec3Of(pp.frame.U())
	v, okV := proofbound.IvVec3Of(pp.frame.V())
	n, okN := proofbound.IvVec3Of(pp.frame.N())
	if !okX || !okY || !okZ || !okT || !okO || !okU || !okV || !okN {
		return survey2d.PlacedFrameMap{}, false
	}
	place := func(local proofbound.IvVec3) proofbound.IvVec3 {
		return proofbound.IvVec3Add(
			proofbound.IvVec3Mul(ex, local[0]),
			proofbound.IvVec3Add(proofbound.IvVec3Mul(ey, local[1]), proofbound.IvVec3Mul(ez, local[2])),
		)
	}
	return survey2d.PlacedFrameMap{
		Origin: proofbound.IvVec3Add(place(origin), translation),
		Du:     place(u),
		Dv:     place(v),
		Dn:     place(n),
	}, true
}

// intervalMid is one rational strictly inside an enclosure, the point a bound
// measured from either end is smallest against.
func intervalMid(a proofbound.RatInterval) *big.Rat {
	return new(big.Rat).Mul(new(big.Rat).Add(a.Lo, a.Hi), big.NewRat(1, 2))
}
