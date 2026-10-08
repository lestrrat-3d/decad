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
	world, ok := survey2d.NewPlacedFrameMap(pl.frame, pl.xform)
	if !ok {
		return capPatchModel{}, false
	}
	return capband.PatchNormalModel(capband.NormalInput{
		SinH: sinH, CosH: cosH, Origin: origin, Axis: axis, Pull: p,
		World: world, CU: g.CU, CV: g.CV, CapZ: g.CapZ,
		Radius: g.CapRadius, Th0: g.Th0, Reversed: f.reversed,
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

// intervalMid is one rational strictly inside an enclosure, the point a bound
// measured from either end is smallest against.
func intervalMid(a proofbound.RatInterval) *big.Rat {
	return new(big.Rat).Mul(new(big.Rat).Add(a.Lo, a.Hi), big.NewRat(1, 2))
}
