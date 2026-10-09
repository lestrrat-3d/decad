package brepgeom

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// FaceMeasure carries one face's neutral record for the divergence sums.
// Region states its published area; RestoredRegion states the region used by
// the volume and moments when a fillet restored the source contour.
type FaceMeasure struct {
	Embed                  Embed
	Planar, Outward        bool
	Z0, Z1                 float64
	RestoredZ0, RestoredZ1 float64
	Z0Delta, Z1Delta       float64
	Delta                  float64
	Region, RestoredRegion Region
	Wall                   sectionrecord.CurveSegment
	Walk                   survey2d.SegmentWalk
}

// FaceMass is the band contribution to three times the volume, the three
// first moments and the surface area.
type FaceMass struct {
	Vol3    proofbound.RatInterval
	Moments [3]proofbound.RatInterval
	Area    proofbound.BoundedScalar
}

// FaceReadings are the bounded readings before the reference frame maps the
// centroid coordinates into the body's world placement.
type FaceReadings struct {
	Volume   proofbound.BoundedScalar
	Area     proofbound.BoundedScalar
	Centroid [3]proofbound.BoundedScalar
}

// IntegrateFaces adds every recorded face and the initial band terms in face
// order. The reader resolves a face when reached, preserving cancellation and
// error order of the body build.
//
// In reference coordinates a planar face at level z with outward sign s
// contributes s·z·A to 3V, and ½·s·σ·z²·A to the first moment along the
// reference axis its normal lands on. A swept face over height h contributes
// 2·h·g to 3V and σ·h·mu, σ·h·mv to its two in-plane moments. These are the
// divergence theorem's volume and first-moment sums over a closed boundary.
// A face's section and level displacements widen volume and moment bounds;
// signed values may cancel but their bound terms accumulate in absolute sums.
//
// The initial mass contains a route L band's strip terms. A fillet band's
// rewritten faces are read through RestoredRegion for volume and moments;
// their published region remains the area reading. The caller includes each
// fillet band's extents separately when it publishes the body's box.
func IntegrateFaces(ctx context.Context, count int, read func(int) (FaceMeasure, error), bands FaceMass,
	coordUpper, sectionDelta, axialDelta float64) (FaceReadings, error) {
	vol3, moments, area := bands.Vol3, bands.Moments, bands.Area
	displaced := 0.0
	for fi := range count {
		if err := ctx.Err(); err != nil {
			return FaceReadings{}, err
		}
		f, err := read(fi)
		if err != nil {
			return FaceReadings{}, err
		}
		z0, z1 := proofarith.FloatRat(f.RestoredZ0), proofarith.FloatRat(f.RestoredZ1)
		if f.Planar {
			s := big.NewRat(-1, 1)
			if f.Outward {
				s = big.NewRat(1, 1)
			}
			vol3 = proofbound.IntervalAdd(vol3,
				proofbound.IntervalScale(f.RestoredRegion.Area, proofbound.RatMul(s, z0)))
			k := f.Embed.Axis[2]
			half := proofbound.RatMul(big.NewRat(1, 2), s, big.NewRat(int64(f.Embed.Sign[2]), 1), z0, z0)
			moments[k] = proofbound.IntervalAdd(moments[k], proofbound.IntervalScale(f.RestoredRegion.Area, half))
			area = proofbound.BoundedAdd(area, f.Region.Published)
			displaced = proofbound.AbsSumUpper(displaced,
				proofbound.ProductUpper(f.Z0Delta,
					proofbound.AbsSumUpper(f.RestoredRegion.Upper, f.RestoredRegion.Displacement)))
			continue
		}
		terms, err := SegmentIntegrals(f.Wall)
		if err != nil {
			return FaceReadings{}, err
		}
		h := new(big.Rat).Sub(z1, z0)
		vol3 = proofbound.IntervalAdd(vol3,
			proofbound.IntervalScale(terms[0], proofbound.RatMul(big.NewRat(2, 1), h)))
		for i, mom := range [2]proofbound.RatInterval{terms[1], terms[2]} {
			scale := proofbound.RatMul(big.NewRat(int64(f.Embed.Sign[i]), 1), h)
			k := f.Embed.Axis[i]
			moments[k] = proofbound.IntervalAdd(moments[k], proofbound.IntervalScale(mom, scale))
		}
		area = proofbound.BoundedAdd(area, WallArea(f.Delta, f.Z0, f.Z1, f.Z0Delta, f.Z1Delta, f.Walk))
		heightUpper := proofbound.AbsSumUpper(proofbound.UpRound(f.RestoredZ1-f.RestoredZ0), f.Z0Delta, f.Z1Delta)
		band := proofbound.SectionDisplacementArea(f.Delta, 1,
			proofbound.AbsSumUpper(f.Walk.Length, f.Walk.LengthBound))
		displaced = proofbound.AbsSumUpper(displaced, proofbound.ProductUpper(heightUpper, band))
	}
	envelope := proofbound.AbsSumUpper(coordUpper, sectionDelta, axialDelta)
	volume := proofbound.IntervalScale(vol3, big.NewRat(1, 3))
	heldVolume := Held(volume)
	if !(heldVolume > 0) {
		return FaceReadings{}, fmt.Errorf(`%w: a brep body encloses no volume`, decaderr.ErrDegenerate)
	}
	volumeScalar := proofbound.MeasuredScalar(heldVolume,
		proofbound.AbsSumUpper(proofbound.IntervalFloatError(volume, heldVolume), displaced))
	readings := FaceReadings{Volume: volumeScalar, Area: area}
	exact := displaced == 0 && volume.Lo.Cmp(volume.Hi) == 0
	for i := range moments {
		exact = exact && moments[i].Lo.Cmp(moments[i].Hi) == 0
	}
	for i, mom := range moments {
		if exact {
			q := new(big.Rat).Quo(mom.Lo, volume.Lo)
			held, _ := q.Float64()
			readings.Centroid[i] = proofbound.MeasuredScalar(held, proofarith.RationalFloatError(q, held))
			continue
		}
		heldMoment := Held(mom)
		momBound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(mom, heldMoment),
			proofbound.ProductUpper(displaced, envelope))
		readings.Centroid[i] = proofbound.BoundedDiv(proofbound.MeasuredScalar(heldMoment, momBound), volumeScalar)
	}
	return readings, nil
}
