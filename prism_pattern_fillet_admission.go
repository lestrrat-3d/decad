package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/prismpattern"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Selected chains must be authenticated source-root generators. A faceted
// edge with the same held endpoints is insufficient without that source map.
func prismPatternFilletContactsOf(ctx context.Context, body *Body, edges []*Edge,
	radius float64) ([]prismPatternFilletContact, error) {
	fp, ok := body.n.Payload.(facetedPayload)
	if !ok || fp.PrismPatternJoin == nil || radius <= 0 || proofbound.IsNonFinite(radius) {
		return nil, fmt.Errorf("%w: fillet has no certified joined-prism source or radius", ErrUnsupported)
	}
	rec := fp.PrismPatternJoin
	seen := make(map[[2]int]struct{}, len(edges))
	contacts := make([]prismPatternFilletContact, 0, len(edges))
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
				contact, err := prismPatternFilletContactOf(part, i, station,
					rec.Source.RootArc, rec.Source.RootIndex, rec.Source.BlankRadius, radius)
				if err != nil {
					return nil, err
				}
				contact.ringCut, err = prismPatternFilletRingCutOf(rec, contact)
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
	if err := prismPatternFilletClearance(ctx, rec, contacts, radius); err != nil {
		return nil, err
	}
	return contacts, nil
}

// Nonconnector source walls clear the complete fillet circle radially. Other
// connectors and fillet arcs must clear each contact's angular sector.
func prismPatternFilletClearance(ctx context.Context, rec *prismPatternJoinRecord,
	contacts []prismPatternFilletContact, radius float64) error {
	reach := proofbound.AbsSumUpper(rec.Source.BlankRadius,
		proofbound.ProductUpper(2, radius))
	minScale2 := big.NewRat(1, 1)
	for _, part := range rec.Parts {
		scale2 := prismpattern.Scale2(part.Prism.Xform)
		if scale2.Cmp(minScale2) < 0 {
			minScale2 = scale2
		}
	}
	reachSq := new(big.Rat).Quo(prismpattern.RadiusSq(reach), minScale2)
	if err := prismpattern.OutsideRoot(ctx, rec.Source.Section.Profile.Outer.Segments,
		rec.Source.RootIndex, rec.Source.RootArc, reachSq); err != nil {
		return fmt.Errorf("%w: joined-prism fillet radial envelope reaches another source wall: %w",
			ErrUnsupported, err)
	}
	for _, part := range rec.Parts {
		scale2 := prismpattern.Scale2(part.Prism.Xform)
		for i, segment := range part.Prism.Profile.Outer.Segments {
			arc, ok := segment.(arcSeg)
			if !ok || i == rec.Source.RootIndex {
				continue
			}
			nominalSq := new(big.Rat).Mul(scale2, prismpattern.RadialSq(arc.Start))
			lower := proofarith.FloatRat(proofbound.RatSqrtDown(nominalSq))
			residual := proofbound.ProductUpper(proofbound.RatSqrtUp(scale2),
				circularbounds.ArcRadialResidualUpper(arc))
			lower.Sub(lower, proofarith.FloatRat(residual))
			if lower.Cmp(proofarith.FloatRat(reach)) <= 0 {
				return fmt.Errorf("%w: joined-prism fillet arc %d has no radial clearance",
					ErrUnsupported, i)
			}
		}
	}
	sectors := make([]prismPatternFilletSector, len(contacts))
	for i, contact := range contacts {
		sector, err := prismPatternFilletSectorOf(rec, contact, radius)
		if err != nil {
			return err
		}
		sectors[i] = sector
		for j := range i {
			if !prismPatternFilletSectorsSeparate(sector, sectors[j]) {
				return fmt.Errorf("%w: joined-prism fillet contact sectors overlap", ErrUnsupported)
			}
		}
	}
	for i, contact := range contacts {
		part := rec.Parts[contact.part]
		station := contact.station
		if err := ctx.Err(); err != nil {
			return err
		}
		rootPoint := rec.Source.RootArc.Start
		if station == part.RootStart {
			rootPoint = rec.Source.RootArc.End
		}
		root := prismPatternExactPlacedPoint(part, rootPoint)
		held := part.Mesh.Vertices[2*station]
		rootError := proofbound.AbsSumUpper(
			proofbound.IntervalFloatError(root.X, held.X),
			proofbound.IntervalFloatError(root.Y, held.Y))
		if rootError > units.Millimeters(part.Mesh.Bound).Base() {
			return fmt.Errorf("%w: joined-prism fillet contact %d lost its source station", ErrUnsupported, i)
		}
		for j, other := range rec.Parts {
			for _, connector := range []struct {
				index, station int
			}{{0, 0}, {rec.Source.RootIndex - 1, other.RootStart}} {
				if contact.part == j && station == connector.station {
					continue
				}
				line, ok := other.Prism.Profile.Outer.Segments[connector.index].(lineSeg)
				if !ok {
					return fmt.Errorf("%w: joined-prism fillet lost a source connector", ErrUnsupported)
				}
				connectorSector, err := prismPatternFilletConnectorSector(other, line)
				if err != nil || !prismPatternFilletSectorsSeparate(sectors[i], connectorSector) {
					return fmt.Errorf("%w: joined-prism fillet contact %d reaches connector %d segment %d",
						ErrUnsupported, i, j, connector.index)
				}
			}
		}
	}
	return nil
}
