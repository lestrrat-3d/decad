package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// chamferCapRadialRun measures only the part of the inset rim in a radial
// uncertainty band around the source root. Complete cap source coverage is
// checked separately.
func chamferCapRadialRun(verts []r3.Vec, ring []int,
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

// chamferCapTooth is one source tooth's exact cap contour and its departure bound.
type chamferCapTooth struct {
	points []XY
	poly   []meshbool.Xp2
	bound2 *big.Rat
}

// chamferCapContourOf returns one source tooth's exact cap contour and its departure bound.
func chamferCapContourOf(verts []r3.Vec, facetDeparture float64, capIndex int, z float64) (chamferCapTooth, error) {
	n := len(verts) / 2
	if n < 6 || n > 4096 {
		return chamferCapTooth{}, fmt.Errorf("%w: cap tooth contour exceeds its work limit", decaderr.ErrUnsupported)
	}
	first := capIndex * n
	result := chamferCapTooth{
		points: make([]XY, n), poly: make([]meshbool.Xp2, n),
		bound2: proofarith.MulRat(new(big.Rat), proofarith.FloatRat(facetDeparture),
			proofarith.FloatRat(facetDeparture)),
	}
	for i, p := range verts[first : first+n] {
		if p.Z != z {
			return chamferCapTooth{}, fmt.Errorf("%w: source tooth cap leaves the blank plane", decaderr.ErrUnsupported)
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

// chamferExteriorRimTeeth checks each complete inset edge against the source
// tooth contours. Strict chord clearance keeps it away from the moving source
// curve, and winding at its midpoint fixes which side remains occupied. This
// checks the inset rim only. The cap interior and root band are separate.
func chamferExteriorRimTeeth(ctx context.Context, teeth []chamferCapTooth,
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

// ChamferCapRadialReach measures only the part of the inset rim in a radial
// uncertainty band around the source root. Complete cap source coverage is
// checked separately.
func ChamferCapRadialReach(source ChamferSource,
	verts []r3.Vec, ring []int, capIndex int, outerRadius float64) (float64, error) {
	if source.BlankRadius <= 0 || proofbound.IsNonFinite(source.BlankRadius) ||
		proofbound.IsNonFinite(outerRadius) || capIndex < 0 || capIndex > 1 ||
		outerRadius <= source.BlankRadius {
		return 0, fmt.Errorf("%w: cap radial band has invalid limits", decaderr.ErrUnsupported)
	}
	outer := proofarith.FloatRat(outerRadius)
	outer2 := new(big.Rat).Mul(outer, outer)
	for _, tooth := range source.Teeth {
		if tooth == nil {
			return 0, fmt.Errorf("%w: cap radial band lacks a source tooth", decaderr.ErrUnsupported)
		}
		lp, ok := tooth.Payload.(loftrecord.Payload)
		if !ok || !lp.AuthenticatedReconstruction {
			return 0, fmt.Errorf("%w: cap slit lacks an authenticated tooth", decaderr.ErrUnsupported)
		}
		profile := lp.Profile0
		if capIndex == 1 {
			profile = lp.Profile1
		}
		if len(profile.Outer.Segments) <= 5 {
			return 0, fmt.Errorf("%w: cap slit lacks the source root arc", decaderr.ErrUnsupported)
		}
		arc, ok := profile.Outer.Segments[5].(sectionrecord.ArcSeg)
		if !ok {
			return 0, fmt.Errorf("%w: cap slit lacks the source root arc", decaderr.ErrUnsupported)
		}
		p := SourcePoint(lp.Xform.Basis().EX, arc.Start)
		if q := SourceDot(p, p); q.Cmp(outer2) > 0 {
			return 0, fmt.Errorf("%w: cap radial band misses a source root", decaderr.ErrUnsupported)
		}
	}
	return chamferCapRadialRun(verts, ring, source.BlankRadius, source.BoreRadius, outerRadius)
}

// ChamferExteriorLineage matches every true tooth cap contour outside the
// centered root arc to a retained held chord. Union copied these chords, in order, into its audited
// outer cap loop. A later inset whose exact clearance from that whole loop
// exceeds the Loft departure cannot be crossed by any such source contour.
func ChamferExteriorLineage(fp facetedrecord.Payload,
	source ChamferSource, capIndex int) error {
	if capIndex < 0 || capIndex > 1 || len(source.Outer[capIndex]) < 3 ||
		fp.MeshBound < 0 || proofbound.IsNonFinite(fp.MeshBound) {
		return fmt.Errorf("%w: cap has no exterior contour correspondence", decaderr.ErrUnsupported)
	}
	outer := source.Outer[capIndex]
	edges := make(map[[2]r3.Vec]struct{}, len(outer))
	for i, index := range outer {
		next := outer[(i+1)%len(outer)]
		if index < 0 || index >= len(fp.Verts) || next < 0 || next >= len(fp.Verts) {
			return fmt.Errorf("%w: cap outer loop has invalid indices", decaderr.ErrUnsupported)
		}
		edges[[2]r3.Vec{fp.Verts[index], fp.Verts[next]}] = struct{}{}
	}
	for _, body := range source.Teeth {
		if body == nil {
			return fmt.Errorf("%w: cap lacks an original tooth", decaderr.ErrUnsupported)
		}
		lp, ok := body.Payload.(loftrecord.Payload)
		if !ok || !lp.AuthenticatedReconstruction || lp.Proof.FacetDeparture < 0 ||
			lp.Proof.FacetDeparture > fp.MeshBound || len(lp.Verts)%2 != 0 ||
			len(lp.Segment) != 1 {
			return fmt.Errorf("%w: cap tooth has no bounded source contour", decaderr.ErrUnsupported)
		}
		n := len(lp.Verts) / 2
		if len(lp.Segment[0]) != n {
			return fmt.Errorf("%w: cap tooth has no station-to-curve map", decaderr.ErrUnsupported)
		}
		root := -1
		for j, segment := range lp.Segment[0] {
			if segment == 5 {
				root = j
				break
			}
			if segment < 0 || segment > 4 {
				return fmt.Errorf("%w: cap tooth exterior has an unexpected source curve", decaderr.ErrUnsupported)
			}
		}
		if root < 5 || root >= n-1 {
			return fmt.Errorf("%w: cap tooth has no separate root arc", decaderr.ErrUnsupported)
		}
		first := capIndex * n
		for j := range root {
			if _, ok := edges[[2]r3.Vec{lp.Verts[first+j], lp.Verts[first+j+1]}]; !ok {
				return fmt.Errorf("%w: a source tooth exterior chord is absent from the held cap", decaderr.ErrUnsupported)
			}
		}
	}
	return nil
}

func chamferCapToothOf(body *topo.Body, capIndex int, z float64) (chamferCapTooth, error) {
	if body == nil {
		return chamferCapTooth{}, fmt.Errorf("%w: cap lacks a source tooth", decaderr.ErrUnsupported)
	}
	lp, ok := body.Payload.(loftrecord.Payload)
	if !ok || !lp.AuthenticatedReconstruction || len(lp.Verts)%2 != 0 ||
		lp.Proof.FacetDeparture < 0 || proofbound.IsNonFinite(lp.Proof.FacetDeparture) {
		return chamferCapTooth{}, fmt.Errorf("%w: cap tooth has no matched Loft contour", decaderr.ErrUnsupported)
	}
	return chamferCapContourOf(lp.Verts, lp.Proof.FacetDeparture, capIndex, z)
}

// ChamferExteriorRimInside checks the inset rim against each source tooth's
// cap. Each source tooth's held cap chords and exact cap curve have the same
// authenticated parameters and a two-sided FacetDeparture bound. Strict
// chord clearance keeps a complete inset edge away from the moving source
// curve, and winding at its midpoint fixes which side remains occupied.
// This checks the inset rim only. The cap interior and root band are separate.
func ChamferExteriorRimInside(ctx context.Context, source ChamferSource,
	verts []r3.Vec, ring []int, capIndex int, outerRadius float64) error {
	if len(ring) < 3 || len(ring) > 4096 || capIndex < 0 || capIndex > 1 ||
		len(source.Teeth) == 0 || len(source.Teeth) > 64 ||
		source.BlankRadius <= 0 || proofbound.IsNonFinite(source.BlankRadius) ||
		outerRadius <= source.BlankRadius || proofbound.IsNonFinite(outerRadius) {
		return fmt.Errorf("%w: cap exterior source has invalid limits", decaderr.ErrUnsupported)
	}
	z := source.Z0
	if capIndex == 1 {
		z = source.Z1
	}
	teeth := make([]chamferCapTooth, len(source.Teeth))
	for i, body := range source.Teeth {
		tooth, err := chamferCapToothOf(body, capIndex, z)
		if err != nil {
			return err
		}
		teeth[i] = tooth
	}
	return chamferExteriorRimTeeth(ctx, teeth, verts, ring, source.BlankRadius, outerRadius)
}
