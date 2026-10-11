package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This measures only the part of the inset rim in a radial uncertainty band
// around the source root. Complete cap source coverage is checked separately.
func loftCylinderChamferCapRadialReach(source loftCylinderChamferSource,
	verts []r3.Vec, ring []int, capIndex int, outerRadius float64) (float64, error) {
	if source.blankRadius <= 0 || proofbound.IsNonFinite(source.blankRadius) ||
		proofbound.IsNonFinite(outerRadius) || capIndex < 0 || capIndex > 1 ||
		outerRadius <= source.blankRadius {
		return 0, fmt.Errorf("%w: cap radial band has invalid limits", ErrUnsupported)
	}
	radius := proofarith.FloatRat(source.blankRadius)
	r2 := new(big.Rat).Mul(radius, radius)
	outer := proofarith.FloatRat(outerRadius)
	outer2 := new(big.Rat).Mul(outer, outer)
	for _, tooth := range source.teeth {
		if tooth == nil {
			return 0, fmt.Errorf("%w: cap radial band lacks a source tooth", ErrUnsupported)
		}
		lp, ok := tooth.payload.(loftPayload)
		if !ok || !lp.authenticatedReconstruction {
			return 0, fmt.Errorf("%w: cap slit lacks an authenticated tooth", ErrUnsupported)
		}
		profile := lp.profile0
		if capIndex == 1 {
			profile = lp.profile1
		}
		if len(profile.Outer.Segments) <= 5 {
			return 0, fmt.Errorf("%w: cap slit lacks the source root arc", ErrUnsupported)
		}
		arc, ok := profile.Outer.Segments[5].(arcSeg)
		if !ok {
			return 0, fmt.Errorf("%w: cap slit lacks the source root arc", ErrUnsupported)
		}
		p := loftCylinderSourcePoint(lp, arc.Start)
		if q := loftCylinderSourceDot(p, p); q.Cmp(outer2) > 0 {
			return 0, fmt.Errorf("%w: cap radial band misses a source root", ErrUnsupported)
		}
	}
	if len(ring) < 3 || len(ring) > 4_096 {
		return 0, fmt.Errorf("%w: cap slit ring exceeds its work limit", ErrUnsupported)
	}
	gap2 := new(big.Rat).Sub(outer2, r2)
	width := proofbound.RatFloatUp(new(big.Rat).Quo(gap2,
		new(big.Rat).Mul(big.NewRat(2, 1), radius)))
	if source.boreRadius < 0 || proofbound.IsNonFinite(source.boreRadius) ||
		proofbound.AbsSumUpper(source.boreRadius, width) >= source.blankRadius {
		return 0, fmt.Errorf("%w: cap radial path reaches the source bore", ErrUnsupported)
	}
	hit := make([]bool, len(ring))
	lengths := make([]float64, len(ring))
	radii := make([]*big.Rat, len(ring))
	for i, index := range ring {
		next := ring[(i+1)%len(ring)]
		a := loftCylinderExactXY(verts[index])
		b := loftCylinderExactXY(verts[next])
		qa, qb := loftCylinderSourceDot(a, a), loftCylinderSourceDot(b, b)
		radii[i] = qa
		if qa.Cmp(r2) == 0 || qa.Cmp(outer2) == 0 {
			return 0, fmt.Errorf("%w: cap radial band meets an inset vertex", ErrUnsupported)
		}
		max2 := qa
		if qb.Cmp(max2) > 0 {
			max2 = qb
		}
		if loftCylinderSourceRadiusMin2(a, b).Cmp(outer2) >= 0 || max2.Cmp(r2) <= 0 {
			continue
		}
		hit[i] = true
		d := loftCylinderSourceSub(b, a)
		len2 := loftCylinderSourceDot(d, d)
		der0 := new(big.Rat).Mul(big.NewRat(2, 1), loftCylinderSourceDot(a, d))
		der1 := new(big.Rat).Add(der0, new(big.Rat).Mul(big.NewRat(2, 1), len2))
		if der0.Sign() == 0 || der0.Sign() != der1.Sign() {
			return 0, fmt.Errorf("%w: cap radial-band edge has a tangent radius", ErrUnsupported)
		}
		minDer := new(big.Rat).Abs(der0)
		if abs := new(big.Rat).Abs(der1); abs.Cmp(minDer) < 0 {
			minDer = abs
		}
		lengths[i] = proofbound.ProductUpper(proofbound.RatSqrtUp(len2),
			proofbound.RatFloatUp(new(big.Rat).Quo(gap2, minDer)))
	}
	start := -1
	for i, yes := range hit {
		if !yes {
			start = i
			break
		}
	}
	if start < 0 {
		return 0, fmt.Errorf("%w: cap radial band covers the complete inset", ErrUnsupported)
	}
	maxRun, run := 0.0, 0.0
	first := -1
	for step := 1; step <= len(hit); step++ {
		i := (start + step) % len(hit)
		if hit[i] {
			if first < 0 {
				first = i
			} else if radii[i].Cmp(r2) < 0 || radii[i].Cmp(outer2) > 0 {
				return 0, fmt.Errorf("%w: cap radial-band rim run is disconnected", ErrUnsupported)
			}
			run = proofbound.AbsSumUpper(run, lengths[i])
			continue
		}
		if first < 0 {
			continue
		}
		if radii[first].Cmp(r2) >= 0 && radii[i].Cmp(r2) >= 0 {
			return 0, fmt.Errorf("%w: cap radial-band rim run lacks blank material", ErrUnsupported)
		}
		if proofbound.IsNonFinite(run) {
			return 0, fmt.Errorf("%w: cap radial-band rim run is unbounded", ErrUnsupported)
		}
		maxRun = max(maxRun, run)
		run, first = 0, -1
	}
	reach := proofbound.AbsSumUpper(width, maxRun)
	if proofbound.IsNonFinite(reach) || reach >= 0.1 {
		return 0, fmt.Errorf("%w: cap radial band exceeds the source-boundary allowance", ErrUnsupported)
	}
	return reach, nil
}

func loftCylinderChamferPointSegmentGap2(p, a, b loftCylinderXY) *big.Rat {
	d := loftCylinderSourceSub(b, a)
	den := loftCylinderSourceDot(d, d)
	if den.Sign() == 0 {
		return nil
	}
	t := new(big.Rat).Quo(loftCylinderSourceDot(loftCylinderSourceSub(p, a), d), den)
	if t.Sign() < 0 {
		t.SetInt64(0)
	} else if t.Cmp(big.NewRat(1, 1)) > 0 {
		t.SetInt64(1)
	}
	gap := loftCylinderSourceSub(p, loftCylinderSourceLerp(a, b, t))
	return loftCylinderSourceDot(gap, gap)
}

func loftCylinderChamferSegmentGapAbove(a, b, c, d loftCylinderXY, bound2 *big.Rat) bool {
	if loftCylinderEdgesMeet(a, b, c, d) {
		return false
	}
	for _, v := range [4]*big.Rat{
		loftCylinderChamferPointSegmentGap2(a, c, d),
		loftCylinderChamferPointSegmentGap2(b, c, d),
		loftCylinderChamferPointSegmentGap2(c, a, b),
		loftCylinderChamferPointSegmentGap2(d, a, b),
	} {
		if v == nil || v.Cmp(bound2) <= 0 {
			return false
		}
	}
	return true
}

// The loop corners are exact dyadic floats. One downward float step makes a
// positive AABB gap a lower bound on their exact coordinate separation.
func loftCylinderChamferBoxesApart(a, b, c, d r3.Vec, bound float64) bool {
	return math.Nextafter(min(c.X, d.X)-max(a.X, b.X), math.Inf(-1)) > bound ||
		math.Nextafter(min(a.X, b.X)-max(c.X, d.X), math.Inf(-1)) > bound ||
		math.Nextafter(min(c.Y, d.Y)-max(a.Y, b.Y), math.Inf(-1)) > bound ||
		math.Nextafter(min(a.Y, b.Y)-max(c.Y, d.Y), math.Inf(-1)) > bound
}

// This gate certifies only clearance between two held cap loops. Source-cap
// coverage also needs the original tooth contours and root seam certificate.
func loftCylinderChamferHeldOutlineGap(ctx context.Context, verts []r3.Vec,
	outer, inner []int, bound float64) error {
	if len(outer) < 3 || len(inner) < 3 || bound < 0 || proofbound.IsNonFinite(bound) ||
		len(outer)*len(inner) > 4_194_304 {
		return fmt.Errorf("%w: cap outline clearance exceeds its work limit", ErrUnsupported)
	}
	old := make([]loftCylinderXY, len(outer))
	newLoop := make([]loftCylinderXY, len(inner))
	for i, index := range outer {
		old[i] = loftCylinderExactXY(verts[index])
	}
	for i, index := range inner {
		newLoop[i] = loftCylinderExactXY(verts[index])
	}
	boundRat := proofarith.FloatRat(bound)
	bound2 := new(big.Rat).Mul(boundRat, boundRat)
	for i, a := range outer {
		b := outer[(i+1)%len(outer)]
		for j, c := range inner {
			if j%64 == 0 {
				if err := ctx.Err(); err != nil {
					return err
				}
			}
			d := inner[(j+1)%len(inner)]
			if loftCylinderChamferBoxesApart(verts[a], verts[b], verts[c], verts[d], bound) {
				continue
			}
			if !loftCylinderChamferSegmentGapAbove(old[i], old[(i+1)%len(old)],
				newLoop[j], newLoop[(j+1)%len(newLoop)], bound2) {
				return fmt.Errorf("%w: cap inset lies within source-outline allowance at edges %d/%d",
					ErrUnsupported, i, j)
			}
		}
	}
	return nil
}
