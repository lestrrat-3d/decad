package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
)

// revolveLoftJoinRecord retains one proved section/sector source and the
// bounds published by its joined mesh. Later placed sectors and cap-ring
// rewrites can authenticate against the same immutable source.
type revolveLoftJoinRecord struct {
	certificate                           *revolveLoftSectorCertificate
	blank                                 *Body
	first                                 facetedPayload
	placed                                []facetedPayload
	capRing                               *revolvedSectionRing
	boundaryBound, volumeBound, areaBound float64
}

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
			cert, ok := certifyRevolveLoftSector(blank, first)
			if !ok {
				continue
			}
			path, bound, err := revolveExteriorPath(cert.base, cert.boundary,
				cert.trim.base.pointSection.source, cert.section)
			if err != nil {
				return nil, true, err
			}
			lo, hi, ok := revolveRootAngles(path)
			if !ok {
				return nil, true, ErrUnsupported
			}
			cert.sector, cert.crossingBound = [2]float64{lo, hi}, bound
			cert.loop = path
			sources := make([]facetedPayload, 0, len(operands)-1)
			sources = append(sources, cert.trimmed)
			for i, operand := range operands {
				if i == blankIndex || i == firstIndex {
					continue
				}
				placed, ok := operand.n.Payload.(facetedPayload)
				if !ok || placed.revolveLoftSector == nil ||
					placed.revolveLoftSector.source != cert.trim ||
					placed.xform != placed.revolveLoftSector.motion {
					return nil, true, fmt.Errorf("%w: placed sector lost its source", ErrUnsupported)
				}
				sources = append(sources, placed)
			}
			if !revolveSectorSourcesSeparate(sources) {
				return nil, true, fmt.Errorf("%w: complete sector sources are not separate", ErrUnsupported)
			}
			body, ring, err := buildRevolveLoftSectorsJoin(ctx, d, ref, cert, sources)
			if err != nil {
				return nil, true, err
			}
			payload, ok := body.n.Payload.(facetedPayload)
			if !ok {
				return nil, true, ErrUnsupported
			}
			payload.revolveLoftJoin = &revolveLoftJoinRecord{certificate: cert,
				blank: blank, first: sources[0],
				placed: append([]facetedPayload(nil), sources[1:]...), capRing: ring,
				boundaryBound: payload.meshBound, volumeBound: payload.volSymDiff,
				areaBound: payload.areaSlack}
			body.n.Payload = payload
			return body, true, nil
		}
	}
	return nil, false, nil
}

type revolveLoftSectorPlacement struct {
	source *pointConeTrimRecord
	motion r3.Transform
}

// tryRevolveLoftSectorUnion authenticates source geometry before construction.
func tryRevolveLoftSectorUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	blank, sector := a, b
	cert, ok := certifyRevolveLoftSector(blank, sector)
	if !ok {
		blank, sector = b, a
		cert, ok = certifyRevolveLoftSector(blank, sector)
		if !ok {
			return nil, false, nil
		}
	}
	path, bound, err := revolveExteriorPath(cert.base, cert.boundary,
		cert.trim.base.pointSection.source, cert.section)
	if err != nil {
		return nil, true, err
	}
	lo, hi, ok := revolveRootAngles(path)
	if !ok {
		return nil, true, ErrUnsupported
	}
	cert.sector, cert.crossingBound = [2]float64{lo, hi}, bound
	cert.loop = path
	body, ring, err := buildRevolveLoftSectorsJoin(ctx, d, ref, cert,
		[]facetedPayload{cert.trimmed})
	if err != nil {
		return nil, true, err
	}
	payload, ok := body.n.Payload.(facetedPayload)
	if !ok {
		return nil, true, ErrUnsupported
	}
	payload.revolveLoftJoin = &revolveLoftJoinRecord{certificate: cert, blank: blank,
		first: cert.trimmed, capRing: ring, boundaryBound: payload.meshBound,
		volumeBound: payload.volSymDiff, areaBound: payload.areaSlack}
	body.n.Payload = payload
	return body, true, nil
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
	if !ok || jp.revolveLoftJoin == nil {
		joined, placed = b, a
		jp, ok = joined.n.Payload.(facetedPayload)
	}
	p, placedOK := placed.n.Payload.(facetedPayload)
	if !ok || !placedOK || jp.revolveLoftJoin == nil ||
		jp.xform != r3.Identity() || p.revolveLoftSector == nil ||
		p.revolveLoftSector.source == nil ||
		jp.revolveLoftJoin.certificate == nil ||
		jp.revolveLoftJoin.certificate.trim != p.revolveLoftSector.source ||
		p.xform != p.revolveLoftSector.motion ||
		!massmoment.XAxisRotation(p.revolveLoftSector.motion) {
		return nil, false, nil
	}
	record := jp.revolveLoftJoin
	sectors := make([]facetedPayload, 0, len(record.placed)+2)
	sectors = append(sectors, record.first)
	sectors = append(sectors, record.placed...)
	sectors = append(sectors, p)
	if !revolveSectorSourcesSeparate(sectors) {
		return nil, false, nil
	}
	body, ring, err := buildRevolveLoftSectorsJoin(ctx, d, ref, record.certificate, sectors)
	if err != nil {
		return nil, true, err
	}
	result, ok := body.n.Payload.(facetedPayload)
	if !ok {
		return nil, true, ErrUnsupported
	}
	result.revolveLoftJoin = &revolveLoftJoinRecord{certificate: record.certificate,
		blank: record.blank, first: record.first,
		placed: append([]facetedPayload(nil), sectors[1:]...), capRing: ring,
		boundaryBound: result.meshBound, volumeBound: result.volSymDiff,
		areaBound: result.areaSlack}
	body.n.Payload = result
	return body, true, nil
}
