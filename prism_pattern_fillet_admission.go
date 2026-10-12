package decad

import (
	"context"
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Selected chains must be authenticated source-root generators. A faceted
// edge with the same held endpoints is insufficient without that source map.
func prismPatternFilletContactsOf(ctx context.Context, body *Body, edges []*Edge,
	radius float64) ([]prismPatternFilletContact, error) {
	fp, ok := body.payload.(facetedPayload)
	if !ok || fp.prismPatternJoin == nil || radius <= 0 || proofbound.IsNonFinite(radius) {
		return nil, fmt.Errorf("%w: fillet has no certified joined-prism source or radius", ErrUnsupported)
	}
	rec := fp.prismPatternJoin
	if err := prismPatternFilletClearance(ctx, rec, radius, fp.meshBound); err != nil {
		return nil, err
	}
	seen := make(map[[2]int]struct{}, len(edges))
	contacts := make([]prismPatternFilletContact, 0, len(edges))
	for _, edge := range edges {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if edge == nil || !edge.selectorDirectionProven ||
			edge.selectorDirection != (r3.Vec{Z: 1}) && edge.selectorDirection != (r3.Vec{Z: -1}) {
			return nil, fmt.Errorf("%w: joined-prism fillet selection has an unproved axial edge", ErrUnsupported)
		}
		matched := false
		for i, part := range rec.parts {
			for _, station := range []int{0, part.rootStart} {
				bottom := fp.verts[part.vertex[0][station]]
				top := fp.verts[part.vertex[1][station]]
				if (edge.start.position != bottom || edge.end.position != top) &&
					(edge.start.position != top || edge.end.position != bottom) {
					continue
				}
				key := [2]int{i, station}
				if _, duplicate := seen[key]; duplicate {
					return nil, fmt.Errorf("%w: joined-prism fillet selects one root twice", ErrUnsupported)
				}
				contact, err := prismPatternFilletContactOf(part, i, station,
					rec.source.rootArc, rec.source.rootIndex, rec.source.blankRadius, radius)
				if err != nil {
					return nil, err
				}
				if !prismPatternFilletContactFitsRing(rec, contact) {
					return nil, fmt.Errorf("%w: joined-prism fillet circle contact passes its first ring station",
						ErrUnsupported)
				}
				seen[key] = struct{}{}
				contacts = append(contacts, contact)
				matched = true
				break
			}
			if matched {
				break
			}
		}
		if !matched {
			return nil, fmt.Errorf("%w: joined-prism fillet selects no authenticated root", ErrUnsupported)
		}
	}
	return contacts, nil
}

// Nonconnector source walls clear every 3r contact disk. The root disks
// themselves are pairwise separate after charging source mesh displacement.
func prismPatternFilletClearance(ctx context.Context, rec *prismPatternJoinRecord,
	radius, sourceBound float64) error {
	reach := proofbound.AbsSumUpper(rec.source.blankRadius,
		proofbound.ProductUpper(3, radius), sourceBound)
	minScale2 := big.NewRat(1, 1)
	for _, part := range rec.parts {
		scale2 := prismPatternScale2(part.prism.xform)
		if scale2.Cmp(minScale2) < 0 {
			minScale2 = scale2
		}
	}
	reachSq := new(big.Rat).Quo(prismPatternRadiusSq(reach), minScale2)
	if err := prismPatternOutsideRoot(ctx, rec.source.section.profile.Outer.Segments,
		rec.source.rootIndex, rec.source.rootArc, reachSq); err != nil {
		return fmt.Errorf("%w: joined-prism fillet local disk reaches another source wall: %w",
			ErrUnsupported, err)
	}
	type root struct {
		point r3.Vec
		bound float64
	}
	roots := make([]root, 0, len(rec.parts)*2)
	for _, part := range rec.parts {
		for _, station := range []int{0, part.rootStart} {
			roots = append(roots, root{part.mesh.vertices[2*station], part.mesh.Bound().Base()})
		}
	}
	for i := range roots {
		for j := i + 1; j < len(roots); j++ {
			dx := new(big.Rat).Sub(proofarith.FloatRat(roots[i].point.X), proofarith.FloatRat(roots[j].point.X))
			dy := new(big.Rat).Sub(proofarith.FloatRat(roots[i].point.Y), proofarith.FloatRat(roots[j].point.Y))
			distanceSq := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
			minimum := proofbound.AbsSumUpper(proofbound.ProductUpper(6, radius),
				roots[i].bound, roots[j].bound)
			if distanceSq.Cmp(prismPatternRadiusSq(minimum)) <= 0 {
				return fmt.Errorf("%w: joined-prism fillet root disks are not separate", ErrUnsupported)
			}
		}
	}
	return nil
}

func prismPatternFilletContactFitsRing(rec *prismPatternJoinRecord,
	contact prismPatternFilletContact) bool {
	part := rec.parts[contact.part]
	old := part.vertex[0][contact.station]
	outline := rec.outlines[0]
	position := -1
	for i, index := range outline {
		if index == old {
			position = i
			break
		}
	}
	if position < 0 {
		return false
	}
	q := rec.mesh.verts[old]
	qIV := prismPatternIVPointOf(proofarith.FloatRat(q.X), proofarith.FloatRat(q.Y))
	cross := func(a, b prismPatternIVPoint) proofbound.RatInterval {
		return proofbound.IntervalSub(proofbound.IntervalMul(a.x, b.y), proofbound.IntervalMul(a.y, b.x))
	}
	if contact.station == 0 {
		previous := rec.mesh.verts[outline[(position+len(outline)-1)%len(outline)]]
		nIV := prismPatternIVPointOf(proofarith.FloatRat(previous.X), proofarith.FloatRat(previous.Y))
		return cross(nIV, contact.circleIV).Lo.Sign() > 0 && cross(contact.circleIV, qIV).Lo.Sign() > 0
	}
	next := rec.mesh.verts[outline[(position+1)%len(outline)]]
	nIV := prismPatternIVPointOf(proofarith.FloatRat(next.X), proofarith.FloatRat(next.Y))
	return cross(qIV, contact.circleIV).Lo.Sign() > 0 && cross(contact.circleIV, nIV).Lo.Sign() > 0
}
