package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type loftCylinderChamferCapTooth struct {
	points []loftCylinderXY
	poly   []meshbool.Xp2
	bound2 *big.Rat
}

// Every true tooth cap contour outside the centered root arc is matched to a
// retained held chord. Union copied these chords, in order, into its audited
// outer cap loop. A later inset whose exact clearance from that whole loop
// exceeds the Loft departure cannot be crossed by any such source contour.
func loftCylinderChamferExteriorLineage(fp facetedPayload,
	source loftCylinderChamferSource, capIndex int) error {
	if capIndex < 0 || capIndex > 1 || len(source.outer[capIndex]) < 3 ||
		fp.meshBound < 0 || proofbound.IsNonFinite(fp.meshBound) {
		return fmt.Errorf("%w: cap has no exterior contour correspondence", ErrUnsupported)
	}
	outer := source.outer[capIndex]
	edges := make(map[[2]r3.Vec]struct{}, len(outer))
	for i, index := range outer {
		next := outer[(i+1)%len(outer)]
		if index < 0 || index >= len(fp.verts) || next < 0 || next >= len(fp.verts) {
			return fmt.Errorf("%w: cap outer loop has invalid indices", ErrUnsupported)
		}
		edges[[2]r3.Vec{fp.verts[index], fp.verts[next]}] = struct{}{}
	}
	for _, body := range source.teeth {
		if body == nil {
			return fmt.Errorf("%w: cap lacks an original tooth", ErrUnsupported)
		}
		lp, ok := body.payload.(loftPayload)
		if !ok || !lp.authenticatedReconstruction || lp.proof.FacetDeparture < 0 ||
			lp.proof.FacetDeparture > fp.meshBound || len(lp.verts)%2 != 0 ||
			len(lp.segment) != 1 {
			return fmt.Errorf("%w: cap tooth has no bounded source contour", ErrUnsupported)
		}
		n := len(lp.verts) / 2
		if len(lp.segment[0]) != n {
			return fmt.Errorf("%w: cap tooth has no station-to-curve map", ErrUnsupported)
		}
		root := -1
		for j, segment := range lp.segment[0] {
			if segment == 5 {
				root = j
				break
			}
			if segment < 0 || segment > 4 {
				return fmt.Errorf("%w: cap tooth exterior has an unexpected source curve", ErrUnsupported)
			}
		}
		if root < 5 || root >= n-1 {
			return fmt.Errorf("%w: cap tooth has no separate root arc", ErrUnsupported)
		}
		first := capIndex * n
		for j := range root {
			if _, ok := edges[[2]r3.Vec{lp.verts[first+j], lp.verts[first+j+1]}]; !ok {
				return fmt.Errorf("%w: a source tooth exterior chord is absent from the held cap", ErrUnsupported)
			}
		}
	}
	return nil
}

func loftCylinderChamferCapToothOf(body *Body, capIndex int, z float64) (loftCylinderChamferCapTooth, error) {
	if body == nil {
		return loftCylinderChamferCapTooth{}, fmt.Errorf("%w: cap lacks a source tooth", ErrUnsupported)
	}
	lp, ok := body.payload.(loftPayload)
	if !ok || !lp.authenticatedReconstruction || len(lp.verts)%2 != 0 ||
		lp.proof.FacetDeparture < 0 || proofbound.IsNonFinite(lp.proof.FacetDeparture) {
		return loftCylinderChamferCapTooth{}, fmt.Errorf("%w: cap tooth has no matched Loft contour", ErrUnsupported)
	}
	n := len(lp.verts) / 2
	if n < 6 || n > 4096 {
		return loftCylinderChamferCapTooth{}, fmt.Errorf("%w: cap tooth contour exceeds its work limit", ErrUnsupported)
	}
	first := capIndex * n
	result := loftCylinderChamferCapTooth{
		points: make([]loftCylinderXY, n), poly: make([]meshbool.Xp2, n),
		bound2: new(big.Rat).Mul(proofarith.FloatRat(lp.proof.FacetDeparture),
			proofarith.FloatRat(lp.proof.FacetDeparture)),
	}
	for i, p := range lp.verts[first : first+n] {
		if p.Z != z {
			return loftCylinderChamferCapTooth{}, fmt.Errorf("%w: source tooth cap leaves the blank plane", ErrUnsupported)
		}
		result.points[i] = loftCylinderExactXY(p)
		result.poly[i] = meshbool.NewXP2(result.points[i].x, result.points[i].y)
	}
	return result, nil
}

// A rational point just below the outer band circle leaves the certified
// exterior part whole. The small overlap makes root-circle bisection error a
// reason to refuse, rather than an uncharged gap in source coverage.
func loftCylinderChamferExteriorPart(a, b loftCylinderXY, outer2, overlap2 *big.Rat) (
	loftCylinderXY, loftCylinderXY, bool, error) {
	qa, qb := loftCylinderSourceDot(a, a), loftCylinderSourceDot(b, b)
	if qa.Cmp(outer2) == 0 || qb.Cmp(outer2) == 0 {
		return loftCylinderXY{}, loftCylinderXY{}, false,
			fmt.Errorf("%w: cap inset touches the radial split vertex", ErrUnsupported)
	}
	max2 := qa
	if qb.Cmp(max2) > 0 {
		max2 = qb
	}
	if max2.Cmp(outer2) < 0 {
		return loftCylinderXY{}, loftCylinderXY{}, false, nil
	}
	if loftCylinderSourceRadiusMin2(a, b).Cmp(outer2) > 0 {
		return a, b, true, nil
	}
	if qa.Cmp(outer2) > 0 && qb.Cmp(outer2) > 0 {
		return loftCylinderXY{}, loftCylinderXY{}, false,
			fmt.Errorf("%w: cap inset returns through its radial split", ErrUnsupported)
	}
	if qa.Cmp(outer2) > 0 {
		a, b = b, a
	}
	if loftCylinderSourceDot(a, a).Cmp(outer2) >= 0 ||
		loftCylinderSourceDot(b, b).Cmp(outer2) <= 0 {
		return loftCylinderXY{}, loftCylinderXY{}, false,
			fmt.Errorf("%w: cap inset has no exterior radial interval", ErrUnsupported)
	}
	d := loftCylinderSourceSub(b, a)
	len2 := loftCylinderSourceDot(d, d)
	der0 := new(big.Rat).Mul(big.NewRat(2, 1), loftCylinderSourceDot(a, d))
	der1 := new(big.Rat).Add(der0, new(big.Rat).Mul(big.NewRat(2, 1), len2))
	if der0.Sign() <= 0 || der1.Sign() <= 0 {
		return loftCylinderXY{}, loftCylinderXY{}, false,
			fmt.Errorf("%w: cap exterior radial interval is not monotone", ErrUnsupported)
	}
	low, high := big.NewRat(0, 1), big.NewRat(1, 1)
	for range 64 {
		mid := new(big.Rat).Quo(new(big.Rat).Add(low, high), big.NewRat(2, 1))
		p := loftCylinderSourceLerp(a, b, mid)
		if loftCylinderSourceDot(p, p).Cmp(outer2) < 0 {
			low = mid
		} else {
			high = mid
		}
	}
	p := loftCylinderSourceLerp(a, b, low)
	if loftCylinderSourceDot(p, p).Cmp(overlap2) <= 0 {
		return loftCylinderXY{}, loftCylinderXY{}, false,
			fmt.Errorf("%w: cap radial split has no certified overlap", ErrUnsupported)
	}
	return p, b, true, nil
}

// Each source tooth's held cap chords and exact cap curve have the same
// authenticated parameters and a two-sided FacetDeparture bound. Strict
// chord clearance keeps a complete inset edge away from the moving source
// curve, and winding at its midpoint fixes which side remains occupied.
// This checks the inset rim only. The cap interior and root band are separate.
func loftCylinderChamferExteriorRimInside(ctx context.Context, source loftCylinderChamferSource,
	verts []r3.Vec, ring []int, capIndex int, outerRadius float64) error {
	if len(ring) < 3 || len(ring) > 4096 || capIndex < 0 || capIndex > 1 ||
		len(source.teeth) == 0 || len(source.teeth) > 64 ||
		source.blankRadius <= 0 || proofbound.IsNonFinite(source.blankRadius) ||
		outerRadius <= source.blankRadius || proofbound.IsNonFinite(outerRadius) {
		return fmt.Errorf("%w: cap exterior source has invalid limits", ErrUnsupported)
	}
	z := source.z0
	if capIndex == 1 {
		z = source.z1
	}
	teeth := make([]loftCylinderChamferCapTooth, len(source.teeth))
	for i, body := range source.teeth {
		tooth, err := loftCylinderChamferCapToothOf(body, capIndex, z)
		if err != nil {
			return err
		}
		teeth[i] = tooth
	}
	outer := proofarith.FloatRat(outerRadius)
	outer2 := new(big.Rat).Mul(outer, outer)
	overlap := proofarith.FloatRat(proofbound.AbsSumUpper(source.blankRadius, 0.009))
	overlap2 := new(big.Rat).Mul(overlap, overlap)
	if overlap.Cmp(outer) >= 0 {
		return fmt.Errorf("%w: cap exterior has no radial overlap", ErrUnsupported)
	}
	pairs := 0
	for i, index := range ring {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		a := loftCylinderExactXY(verts[index])
		b := loftCylinderExactXY(verts[ring[(i+1)%len(ring)]])
		a, b, yes, err := loftCylinderChamferExteriorPart(a, b, outer2, overlap2)
		if err != nil {
			return err
		}
		if !yes {
			continue
		}
		middle := loftCylinderSourceLerp(a, b, big.NewRat(1, 2))
		occupied := false
		for _, tooth := range teeth {
			inside, on, err := meshbool.PointInPoly2(proofbound.NewWorkBudget(ctx), tooth.poly,
				meshbool.NewXP2(middle.x, middle.y))
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
					return fmt.Errorf("%w: cap exterior clearance exceeds its work limit", ErrUnsupported)
				}
				d := tooth.points[(j+1)%len(tooth.points)]
				if !loftCylinderChamferSegmentGapAbove(a, b, c, d, tooth.bound2) {
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
			return fmt.Errorf("%w: cap exterior edge %d has no certified source tooth", ErrUnsupported, i)
		}
	}
	return nil
}
