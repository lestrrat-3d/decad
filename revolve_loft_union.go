package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/r3"
)

// tryRevolveLoftSectorUnionMany admits one Revolve and one authentic point
// Loft source, followed by copies of that same source. Operand order does
// not assign a geometric role; source records do.
func tryRevolveLoftSectorUnionMany(ctx context.Context, d *Document, ref producerID,
	operands []*Body) (*Body, bool, error) {
	for blankIndex, blank := range operands {
		for firstIndex, first := range operands {
			if firstIndex == blankIndex {
				continue
			}
			cert, ok := revolveshell.CertifySector(bodyNode(blank), bodyNode(first))
			if !ok {
				continue
			}
			if err := revolveshell.TraceSectorLoop(cert); err != nil {
				return nil, true, err
			}
			sources := make([]facetedPayload, 0, len(operands)-1)
			sources = append(sources, cert.Trimmed)
			for i, operand := range operands {
				if i == blankIndex || i == firstIndex {
					continue
				}
				placed, ok := operand.n.Payload.(facetedPayload)
				if !ok || placed.RevolveLoftSector == nil ||
					placed.RevolveLoftSector.Source != cert.Trim ||
					placed.Xform != placed.RevolveLoftSector.Motion {
					return nil, true, fmt.Errorf("%w: placed sector lost its source", ErrUnsupported)
				}
				sources = append(sources, placed)
			}
			if !revolveshell.RevolveSectorSourcesSeparate(sources) {
				return nil, true, fmt.Errorf("%w: complete sector sources are not separate", ErrUnsupported)
			}
			body, err := bodyResult(revolveshell.BuildSectorsJoin(ctx, envOf(documentNode(d), ref),
				cert, sources))
			return body, true, err
		}
	}
	return nil, false, nil
}

// tryRevolveLoftSectorUnion authenticates source geometry before construction.
func tryRevolveLoftSectorUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	blank, sector := a, b
	cert, ok := revolveshell.CertifySector(bodyNode(blank), bodyNode(sector))
	if !ok {
		blank, sector = b, a
		cert, ok = revolveshell.CertifySector(bodyNode(blank), bodyNode(sector))
		if !ok {
			return nil, false, nil
		}
	}
	if err := revolveshell.TraceSectorLoop(cert); err != nil {
		return nil, true, err
	}
	body, err := bodyResult(revolveshell.BuildSectorsJoin(ctx, envOf(documentNode(d), ref), cert,
		[]facetedPayload{cert.Trimmed}))
	return body, true, err
}

// tryRevolveLoftNextSectorUnion rebuilds a serial join from the retained
// authenticated Revolve and point-Loft sources. The faceted joined operand
// is not used as a geometric source for the next contact.
func tryRevolveLoftNextSectorUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	joined, placed := a, b
	jp, ok := joined.n.Payload.(facetedPayload)
	if !ok || jp.RevolveLoftJoin == nil {
		joined, placed = b, a
		jp, ok = joined.n.Payload.(facetedPayload)
	}
	p, placedOK := placed.n.Payload.(facetedPayload)
	if !ok || !placedOK || jp.RevolveLoftJoin == nil ||
		jp.Xform != r3.Identity() || p.RevolveLoftSector == nil ||
		p.RevolveLoftSector.Source == nil ||
		jp.RevolveLoftJoin.Certificate == nil ||
		jp.RevolveLoftJoin.Certificate.Trim != p.RevolveLoftSector.Source ||
		p.Xform != p.RevolveLoftSector.Motion ||
		!massmoment.XAxisRotation(p.RevolveLoftSector.Motion) {
		return nil, false, nil
	}
	record := jp.RevolveLoftJoin
	sectors := make([]facetedPayload, 0, len(record.Placed)+2)
	sectors = append(sectors, record.First)
	sectors = append(sectors, record.Placed...)
	sectors = append(sectors, p)
	if !revolveshell.RevolveSectorSourcesSeparate(sectors) {
		return nil, false, nil
	}
	body, err := bodyResult(revolveshell.BuildSectorsJoin(ctx, envOf(documentNode(d), ref),
		record.Certificate, sectors))
	return body, true, err
}
