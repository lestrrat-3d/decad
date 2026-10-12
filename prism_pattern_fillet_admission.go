package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
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
	if err := prismPatternFilletClearance(ctx, rec, contacts, radius); err != nil {
		return nil, err
	}
	return contacts, nil
}

// Nonconnector source walls clear the complete fillet circle radially. Other
// connectors and fillet arcs must clear each contact's angular sector.
func prismPatternFilletClearance(ctx context.Context, rec *prismPatternJoinRecord,
	contacts []prismPatternFilletContact, radius float64) error {
	reach := proofbound.AbsSumUpper(rec.source.blankRadius,
		proofbound.ProductUpper(2, radius))
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
		return fmt.Errorf("%w: joined-prism fillet radial envelope reaches another source wall: %w",
			ErrUnsupported, err)
	}
	for _, part := range rec.parts {
		scale2 := prismPatternScale2(part.prism.xform)
		for i, segment := range part.prism.profile.Outer.Segments {
			arc, ok := segment.(arcSeg)
			if !ok || i == rec.source.rootIndex {
				continue
			}
			nominalSq := new(big.Rat).Mul(scale2, prismPatternRadialSq(arc.Start))
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
		part := rec.parts[contact.part]
		station := contact.station
		if err := ctx.Err(); err != nil {
			return err
		}
		rootPoint := rec.source.rootArc.Start
		if station == part.rootStart {
			rootPoint = rec.source.rootArc.End
		}
		root := prismPatternExactPlacedPoint(part, rootPoint)
		held := part.mesh.vertices[2*station]
		rootError := proofbound.AbsSumUpper(
			proofbound.IntervalFloatError(root.x, held.X),
			proofbound.IntervalFloatError(root.y, held.Y))
		if rootError > part.mesh.Bound().Base() {
			return fmt.Errorf("%w: joined-prism fillet contact %d lost its source station", ErrUnsupported, i)
		}
		for j, other := range rec.parts {
			for _, connector := range []struct {
				index, station int
			}{{0, 0}, {rec.source.rootIndex - 1, other.rootStart}} {
				if contact.part == j && station == connector.station {
					continue
				}
				line, ok := other.prism.profile.Outer.Segments[connector.index].(lineSeg)
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
