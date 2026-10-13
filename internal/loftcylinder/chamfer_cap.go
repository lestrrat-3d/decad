package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ChamferCapRadialRun measures only the part of the inset rim in a radial
// uncertainty band around the source root. Complete cap source coverage is
// checked separately.
func ChamferCapRadialRun(verts []r3.Vec, ring []int,
	blankRadius, boreRadius, outerRadius float64) (float64, error) {
	radius := proofarith.FloatRat(blankRadius)
	r2 := proofarith.MulRat(new(big.Rat), radius, radius)
	outer := proofarith.FloatRat(outerRadius)
	outer2 := proofarith.MulRat(new(big.Rat), outer, outer)
	if len(ring) < 3 || len(ring) > 4_096 {
		return 0, fmt.Errorf("%w: cap slit ring exceeds its work limit", decaderr.ErrUnsupported)
	}
	gap2 := proofarith.SubRat(new(big.Rat), outer2, r2)
	width := proofbound.RatFloatUp(proofarith.QuoRat(new(big.Rat), gap2,
		proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), radius)))
	if boreRadius < 0 || proofbound.IsNonFinite(boreRadius) ||
		proofbound.AbsSumUpper(boreRadius, width) >= blankRadius {
		return 0, fmt.Errorf("%w: cap radial path reaches the source bore", decaderr.ErrUnsupported)
	}
	hit := make([]bool, len(ring))
	lengths := make([]float64, len(ring))
	radii := make([]*big.Rat, len(ring))
	for i, index := range ring {
		next := ring[(i+1)%len(ring)]
		a := ExactXY(verts[index])
		b := ExactXY(verts[next])
		qa, qb := SourceDot(a, a), SourceDot(b, b)
		radii[i] = qa
		if qa.Cmp(r2) == 0 || qa.Cmp(outer2) == 0 {
			return 0, fmt.Errorf("%w: cap radial band meets an inset vertex", decaderr.ErrUnsupported)
		}
		max2 := qa
		if qb.Cmp(max2) > 0 {
			max2 = qb
		}
		if sourceRadiusMin2(a, b).Cmp(outer2) >= 0 || max2.Cmp(r2) <= 0 {
			continue
		}
		hit[i] = true
		d := sourceSub(b, a)
		len2 := SourceDot(d, d)
		der0 := proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), SourceDot(a, d))
		der1 := proofarith.AddRat(new(big.Rat), der0, proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), len2))
		if der0.Sign() == 0 || der0.Sign() != der1.Sign() {
			return 0, fmt.Errorf("%w: cap radial-band edge has a tangent radius", decaderr.ErrUnsupported)
		}
		minDer := new(big.Rat).Abs(der0)
		if abs := new(big.Rat).Abs(der1); abs.Cmp(minDer) < 0 {
			minDer = abs
		}
		lengths[i] = proofbound.ProductUpper(proofbound.RatSqrtUp(len2),
			proofbound.RatFloatUp(proofarith.QuoRat(new(big.Rat), gap2, minDer)))
	}
	start := -1
	for i, yes := range hit {
		if !yes {
			start = i
			break
		}
	}
	if start < 0 {
		return 0, fmt.Errorf("%w: cap radial band covers the complete inset", decaderr.ErrUnsupported)
	}
	maxRun, run := 0.0, 0.0
	first := -1
	for step := 1; step <= len(hit); step++ {
		i := (start + step) % len(hit)
		if hit[i] {
			if first < 0 {
				first = i
			} else if radii[i].Cmp(r2) < 0 || radii[i].Cmp(outer2) > 0 {
				return 0, fmt.Errorf("%w: cap radial-band rim run is disconnected", decaderr.ErrUnsupported)
			}
			run = proofbound.AbsSumUpper(run, lengths[i])
			continue
		}
		if first < 0 {
			continue
		}
		if radii[first].Cmp(r2) >= 0 && radii[i].Cmp(r2) >= 0 {
			return 0, fmt.Errorf("%w: cap radial-band rim run lacks blank material", decaderr.ErrUnsupported)
		}
		if proofbound.IsNonFinite(run) {
			return 0, fmt.Errorf("%w: cap radial-band rim run is unbounded", decaderr.ErrUnsupported)
		}
		maxRun = max(maxRun, run)
		run, first = 0, -1
	}
	reach := proofbound.AbsSumUpper(width, maxRun)
	if proofbound.IsNonFinite(reach) || reach >= 0.1 {
		return 0, fmt.Errorf("%w: cap radial band exceeds the source-boundary allowance", decaderr.ErrUnsupported)
	}
	return reach, nil
}

func chamferPointSegmentGap2(p, a, b XY) *big.Rat {
	d := sourceSub(b, a)
	den := SourceDot(d, d)
	if den.Sign() == 0 {
		return nil
	}
	t := proofarith.QuoRat(new(big.Rat), SourceDot(sourceSub(p, a), d), den)
	if t.Sign() < 0 {
		t.SetInt64(0)
	} else if t.Cmp(big.NewRat(1, 1)) > 0 {
		t.SetInt64(1)
	}
	gap := sourceSub(p, sourceLerp(a, b, t))
	return SourceDot(gap, gap)
}

func chamferSegmentGapAbove(a, b, c, d XY, bound2 *big.Rat) bool {
	if a.X.Cmp(b.X) == 0 && a.Y.Cmp(b.Y) == 0 ||
		c.X.Cmp(d.X) == 0 && c.Y.Cmp(d.Y) == 0 {
		return false
	}
	axisGapAbove := func(a, b, c, d *big.Rat) bool {
		firstMin, firstMax := a, b
		if a.Cmp(b) > 0 {
			firstMin, firstMax = b, a
		}
		secondMin, secondMax := c, d
		if c.Cmp(d) > 0 {
			secondMin, secondMax = d, c
		}
		var high, low *big.Rat
		switch {
		case firstMax.Cmp(secondMin) < 0:
			high, low = secondMin, firstMax
		case secondMax.Cmp(firstMin) < 0:
			high, low = firstMin, secondMax
		default:
			return false
		}
		gap := proofarith.SubRat(new(big.Rat), high, low)
		return proofarith.MulRat(new(big.Rat), gap, gap).Cmp(bound2) > 0
	}
	// An axis gap lower-bounds every distance between the two segments.
	if axisGapAbove(a.X, b.X, c.X, d.X) || axisGapAbove(a.Y, b.Y, c.Y, d.Y) {
		return true
	}
	if EdgesMeet(a, b, c, d) {
		return false
	}
	for _, v := range [4]*big.Rat{
		chamferPointSegmentGap2(a, c, d),
		chamferPointSegmentGap2(b, c, d),
		chamferPointSegmentGap2(c, a, b),
		chamferPointSegmentGap2(d, a, b),
	} {
		if v == nil || v.Cmp(bound2) <= 0 {
			return false
		}
	}
	return true
}

// The loop corners are exact dyadic floats. One downward float step makes a
// positive AABB gap a lower bound on their exact coordinate separation.
func chamferBoxesApart(a, b, c, d r3.Vec, bound float64) bool {
	return math.Nextafter(min(c.X, d.X)-max(a.X, b.X), math.Inf(-1)) > bound ||
		math.Nextafter(min(a.X, b.X)-max(c.X, d.X), math.Inf(-1)) > bound ||
		math.Nextafter(min(c.Y, d.Y)-max(a.Y, b.Y), math.Inf(-1)) > bound ||
		math.Nextafter(min(a.Y, b.Y)-max(c.Y, d.Y), math.Inf(-1)) > bound
}

// ChamferHeldOutlineGap certifies only clearance between two held cap loops.
// Source-cap coverage also needs the original tooth contours and root seam
// certificate.
func ChamferHeldOutlineGap(ctx context.Context, verts []r3.Vec,
	outer, inner []int, bound float64) error {
	if len(outer) < 3 || len(inner) < 3 || bound < 0 || proofbound.IsNonFinite(bound) ||
		len(outer)*len(inner) > 8_000_000 {
		return fmt.Errorf("%w: cap outline clearance exceeds its work limit", decaderr.ErrUnsupported)
	}
	old := make([]XY, len(outer))
	newLoop := make([]XY, len(inner))
	for i, index := range outer {
		old[i] = ExactXY(verts[index])
	}
	for i, index := range inner {
		newLoop[i] = ExactXY(verts[index])
	}
	boundRat := proofarith.FloatRat(bound)
	bound2 := proofarith.MulRat(new(big.Rat), boundRat, boundRat)
	for i, a := range outer {
		b := outer[(i+1)%len(outer)]
		for j, c := range inner {
			if j%64 == 0 {
				if err := ctx.Err(); err != nil {
					return err
				}
			}
			d := inner[(j+1)%len(inner)]
			if chamferBoxesApart(verts[a], verts[b], verts[c], verts[d], bound) {
				continue
			}
			if !chamferSegmentGapAbove(old[i], old[(i+1)%len(old)],
				newLoop[j], newLoop[(j+1)%len(newLoop)], bound2) {
				return fmt.Errorf("%w: cap inset lies within source-outline allowance %g at edges %d/%d",
					decaderr.ErrUnsupported, bound, i, j)
			}
		}
	}
	return nil
}

// ChamferCapTooth is one source tooth's exact cap contour and its departure bound.
type ChamferCapTooth struct {
	points []XY
	poly   []meshbool.Xp2
	bound2 *big.Rat
}

// ChamferCapToothOf returns one source tooth's exact cap contour and its departure bound.
func ChamferCapToothOf(verts []r3.Vec, facetDeparture float64, capIndex int, z float64) (ChamferCapTooth, error) {
	n := len(verts) / 2
	if n < 6 || n > 4096 {
		return ChamferCapTooth{}, fmt.Errorf("%w: cap tooth contour exceeds its work limit", decaderr.ErrUnsupported)
	}
	first := capIndex * n
	result := ChamferCapTooth{
		points: make([]XY, n), poly: make([]meshbool.Xp2, n),
		bound2: proofarith.MulRat(new(big.Rat), proofarith.FloatRat(facetDeparture),
			proofarith.FloatRat(facetDeparture)),
	}
	for i, p := range verts[first : first+n] {
		if p.Z != z {
			return ChamferCapTooth{}, fmt.Errorf("%w: source tooth cap leaves the blank plane", decaderr.ErrUnsupported)
		}
		result.points[i] = ExactXY(p)
		result.poly[i] = meshbool.NewXP2(result.points[i].X, result.points[i].Y)
	}
	return result, nil
}

// A rational point just below the outer band circle leaves the certified
// exterior part whole. The small overlap makes root-circle bisection error a
// reason to refuse, rather than an uncharged gap in source coverage.
func chamferExteriorPart(a, b XY, outer2, overlap2 *big.Rat) (
	XY, XY, bool, error) {
	qa, qb := SourceDot(a, a), SourceDot(b, b)
	if qa.Cmp(outer2) == 0 || qb.Cmp(outer2) == 0 {
		return XY{}, XY{}, false,
			fmt.Errorf("%w: cap inset touches the radial split vertex", decaderr.ErrUnsupported)
	}
	max2 := qa
	if qb.Cmp(max2) > 0 {
		max2 = qb
	}
	if max2.Cmp(outer2) < 0 {
		return XY{}, XY{}, false, nil
	}
	if sourceRadiusMin2(a, b).Cmp(outer2) > 0 {
		return a, b, true, nil
	}
	if qa.Cmp(outer2) > 0 && qb.Cmp(outer2) > 0 {
		return XY{}, XY{}, false,
			fmt.Errorf("%w: cap inset returns through its radial split", decaderr.ErrUnsupported)
	}
	if qa.Cmp(outer2) > 0 {
		a, b = b, a
	}
	if SourceDot(a, a).Cmp(outer2) >= 0 ||
		SourceDot(b, b).Cmp(outer2) <= 0 {
		return XY{}, XY{}, false,
			fmt.Errorf("%w: cap inset has no exterior radial interval", decaderr.ErrUnsupported)
	}
	d := sourceSub(b, a)
	len2 := SourceDot(d, d)
	der0 := proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), SourceDot(a, d))
	der1 := proofarith.AddRat(new(big.Rat), der0, proofarith.MulRat(new(big.Rat), big.NewRat(2, 1), len2))
	if der0.Sign() <= 0 || der1.Sign() <= 0 {
		return XY{}, XY{}, false,
			fmt.Errorf("%w: cap exterior radial interval is not monotone", decaderr.ErrUnsupported)
	}
	low, high := big.NewRat(0, 1), big.NewRat(1, 1)
	for range 64 {
		mid := new(big.Rat).Quo(proofarith.AddRat(new(big.Rat), low, high), big.NewRat(2, 1))
		p := sourceLerp(a, b, mid)
		if SourceDot(p, p).Cmp(outer2) < 0 {
			low = mid
		} else {
			high = mid
		}
	}
	p := sourceLerp(a, b, low)
	if SourceDot(p, p).Cmp(overlap2) <= 0 {
		return XY{}, XY{}, false,
			fmt.Errorf("%w: cap radial split has no certified overlap", decaderr.ErrUnsupported)
	}
	return p, b, true, nil
}

// ChamferExteriorRimInside checks each complete inset edge against the source
// tooth contours. Strict chord clearance keeps it away from the moving source
// curve, and winding at its midpoint fixes which side remains occupied. This
// checks the inset rim only. The cap interior and root band are separate.
func ChamferExteriorRimInside(ctx context.Context, teeth []ChamferCapTooth,
	verts []r3.Vec, ring []int, blankRadius, outerRadius float64) error {
	outer := proofarith.FloatRat(outerRadius)
	outer2 := proofarith.MulRat(new(big.Rat), outer, outer)
	overlap := proofarith.FloatRat(proofbound.AbsSumUpper(blankRadius, 0.009))
	overlap2 := proofarith.MulRat(new(big.Rat), overlap, overlap)
	if overlap.Cmp(outer) >= 0 {
		return fmt.Errorf("%w: cap exterior has no radial overlap", decaderr.ErrUnsupported)
	}
	pairs := 0
	for i, index := range ring {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		a := ExactXY(verts[index])
		b := ExactXY(verts[ring[(i+1)%len(ring)]])
		a, b, yes, err := chamferExteriorPart(a, b, outer2, overlap2)
		if err != nil {
			return err
		}
		if !yes {
			continue
		}
		middle := sourceLerp(a, b, big.NewRat(1, 2))
		occupied := false
		for _, tooth := range teeth {
			inside, on, err := meshbool.PointInPoly2(proofbound.NewWorkBudget(ctx), tooth.poly,
				meshbool.NewXP2(middle.X, middle.Y))
			if err != nil {
				return err
			}
			if !inside || on {
				continue
			}
			separated := true
			for j, c := range tooth.points {
				pairs++
				if pairs > 4_194_304 {
					return fmt.Errorf("%w: cap exterior clearance exceeds its work limit", decaderr.ErrUnsupported)
				}
				d := tooth.points[(j+1)%len(tooth.points)]
				if !chamferSegmentGapAbove(a, b, c, d, tooth.bound2) {
					separated = false
					break
				}
			}
			if separated {
				occupied = true
				break
			}
		}
		if !occupied {
			return fmt.Errorf("%w: cap exterior edge %d has no certified source tooth", decaderr.ErrUnsupported, i)
		}
	}
	return nil
}
