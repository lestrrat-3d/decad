package prismpattern

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// FilletClearance proves the fillet contacts clear every other source wall.
// Nonconnector source walls clear the complete fillet circle radially. Other
// connectors and fillet arcs must clear each contact's angular sector.
func FilletClearance(ctx context.Context, rec *facetedrecord.PrismPatternJoinRecord,
	contacts []FilletContact, radius float64) error {
	reach := proofbound.AbsSumUpper(rec.Source.BlankRadius,
		proofbound.ProductUpper(2, radius))
	minScale2 := big.NewRat(1, 1)
	for _, part := range rec.Parts {
		scale2 := Scale2(part.Prism.Xform)
		if scale2.Cmp(minScale2) < 0 {
			minScale2 = scale2
		}
	}
	reachSq := new(big.Rat).Quo(RadiusSq(reach), minScale2)
	if err := OutsideRoot(ctx, rec.Source.Section.Profile.Outer.Segments,
		rec.Source.RootIndex, rec.Source.RootArc, reachSq); err != nil {
		return fmt.Errorf("%w: joined-prism fillet radial envelope reaches another source wall: %w",
			decaderr.ErrUnsupported, err)
	}
	for _, part := range rec.Parts {
		scale2 := Scale2(part.Prism.Xform)
		for i, segment := range part.Prism.Profile.Outer.Segments {
			arc, ok := segment.(sectionrecord.ArcSeg)
			if !ok || i == rec.Source.RootIndex {
				continue
			}
			nominalSq := new(big.Rat).Mul(scale2, RadialSq(arc.Start))
			lower := proofarith.FloatRat(proofbound.RatSqrtDown(nominalSq))
			residual := proofbound.ProductUpper(proofbound.RatSqrtUp(scale2),
				circularbounds.ArcRadialResidualUpper(arc))
			lower.Sub(lower, proofarith.FloatRat(residual))
			if lower.Cmp(proofarith.FloatRat(reach)) <= 0 {
				return fmt.Errorf("%w: joined-prism fillet arc %d has no radial clearance",
					decaderr.ErrUnsupported, i)
			}
		}
	}
	sectors := make([]filletSector, len(contacts))
	for i, contact := range contacts {
		sector, err := filletSectorOf(rec, contact, radius)
		if err != nil {
			return err
		}
		sectors[i] = sector
		for j := range i {
			if !filletSectorsSeparate(sector, sectors[j]) {
				return fmt.Errorf("%w: joined-prism fillet contact sectors overlap", decaderr.ErrUnsupported)
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
		root := exactPlacedPoint(part, rootPoint)
		held := part.Mesh.Vertices[2*station]
		rootError := proofbound.AbsSumUpper(
			proofbound.IntervalFloatError(root.X, held.X),
			proofbound.IntervalFloatError(root.Y, held.Y))
		if rootError > units.Millimeters(part.Mesh.Bound).Base() {
			return fmt.Errorf("%w: joined-prism fillet contact %d lost its source station", decaderr.ErrUnsupported, i)
		}
		for j, other := range rec.Parts {
			for _, connector := range []struct {
				index, station int
			}{{0, 0}, {rec.Source.RootIndex - 1, other.RootStart}} {
				if contact.part == j && station == connector.station {
					continue
				}
				line, ok := other.Prism.Profile.Outer.Segments[connector.index].(sectionrecord.LineSeg)
				if !ok {
					return fmt.Errorf("%w: joined-prism fillet lost a source connector", decaderr.ErrUnsupported)
				}
				connectorSector, err := filletConnectorSector(other, line)
				if err != nil || !filletSectorsSeparate(sectors[i], connectorSector) {
					return fmt.Errorf("%w: joined-prism fillet contact %d reaches connector %d segment %d",
						decaderr.ErrUnsupported, i, j, connector.index)
				}
			}
		}
	}
	return nil
}
