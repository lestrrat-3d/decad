package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Selected chains must be authenticated source-root generators. A faceted
// edge with the same held endpoints is insufficient without that source map.
func prismPatternFilletContactsOf(ctx context.Context, body *Body, edges []*Edge,
	radius float64) ([]prismpattern.FilletContact, error) {
	fp, ok := body.n.Payload.(facetedPayload)
	if !ok || fp.PrismPatternJoin == nil || radius <= 0 || proofbound.IsNonFinite(radius) {
		return nil, fmt.Errorf("%w: fillet has no certified joined-prism source or radius", ErrUnsupported)
	}
	rec := fp.PrismPatternJoin
	seen := make(map[[2]int]struct{}, len(edges))
	contacts := make([]prismpattern.FilletContact, 0, len(edges))
	for _, edge := range edges {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if edge == nil || !edge.n.SelectorDirectionProven ||
			edge.n.SelectorDirection != (r3.Vec{Z: 1}) && edge.n.SelectorDirection != (r3.Vec{Z: -1}) {
			return nil, fmt.Errorf("%w: joined-prism fillet selection has an unproved axial edge", ErrUnsupported)
		}
		matched := false
		for i, part := range rec.Parts {
			for _, station := range []int{0, part.RootStart} {
				bottom := fp.Verts[part.Vertex[0][station]]
				top := fp.Verts[part.Vertex[1][station]]
				if (edge.n.Start.Position != bottom || edge.n.End.Position != top) &&
					(edge.n.Start.Position != top || edge.n.End.Position != bottom) {
					continue
				}
				key := [2]int{i, station}
				if _, duplicate := seen[key]; duplicate {
					return nil, fmt.Errorf("%w: joined-prism fillet selects one root twice", ErrUnsupported)
				}
				contact, err := prismpattern.FilletContactOf(part, i, station,
					rec.Source.RootArc, rec.Source.RootIndex, rec.Source.BlankRadius, radius)
				if err != nil {
					return nil, err
				}
				contact.RingCut, err = prismpattern.FilletRingCutOf(rec, contact)
				if err != nil {
					return nil, err
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
	if err := prismpattern.FilletClearance(ctx, rec, contacts, radius); err != nil {
		return nil, err
	}
	return contacts, nil
}
