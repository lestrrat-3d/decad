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
	Certificate                           *revolveLoftSectorCertificate
	Blank                                 *Body
	First                                 facetedPayload
	Placed                                []facetedPayload
	CapRing                               *revolvedSectionRing
	BoundaryBound, VolumeBound, AreaBound float64
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
			path, bound, err := revolveExteriorPath(cert.Base, cert.Boundary,
				cert.Trim.base.pointSection.source, cert.Section)
			if err != nil {
				return nil, true, err
			}
			lo, hi, ok := revolveRootAngles(path)
			if !ok {
				return nil, true, ErrUnsupported
			}
			cert.Sector, cert.CrossingBound = [2]float64{lo, hi}, bound
			cert.Loop = path
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
			payload.RevolveLoftJoin = &revolveLoftJoinRecord{Certificate: cert,
				Blank: blank, First: sources[0],
				Placed: append([]facetedPayload(nil), sources[1:]...), CapRing: ring,
				BoundaryBound: payload.MeshBound, VolumeBound: payload.VolSymDiff,
				AreaBound: payload.AreaSlack}
			body.n.Payload = payload
			return body, true, nil
		}
	}
	return nil, false, nil
}

type revolveLoftSectorPlacement struct {
	Source *pointConeTrimRecord
	Motion r3.Transform
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
	path, bound, err := revolveExteriorPath(cert.Base, cert.Boundary,
		cert.Trim.base.pointSection.source, cert.Section)
	if err != nil {
		return nil, true, err
	}
	lo, hi, ok := revolveRootAngles(path)
	if !ok {
		return nil, true, ErrUnsupported
	}
	cert.Sector, cert.CrossingBound = [2]float64{lo, hi}, bound
	cert.Loop = path
	body, ring, err := buildRevolveLoftSectorsJoin(ctx, d, ref, cert,
		[]facetedPayload{cert.Trimmed})
	if err != nil {
		return nil, true, err
	}
	payload, ok := body.n.Payload.(facetedPayload)
	if !ok {
		return nil, true, ErrUnsupported
	}
	payload.RevolveLoftJoin = &revolveLoftJoinRecord{Certificate: cert, Blank: blank,
		First: cert.Trimmed, CapRing: ring, BoundaryBound: payload.MeshBound,
		VolumeBound: payload.VolSymDiff, AreaBound: payload.AreaSlack}
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
	if !revolveSectorSourcesSeparate(sectors) {
		return nil, false, nil
	}
	body, ring, err := buildRevolveLoftSectorsJoin(ctx, d, ref, record.Certificate, sectors)
	if err != nil {
		return nil, true, err
	}
	result, ok := body.n.Payload.(facetedPayload)
	if !ok {
		return nil, true, ErrUnsupported
	}
	result.RevolveLoftJoin = &revolveLoftJoinRecord{Certificate: record.Certificate,
		Blank: record.Blank, First: record.First,
		Placed: append([]facetedPayload(nil), sectors[1:]...), CapRing: ring,
		BoundaryBound: result.MeshBound, VolumeBound: result.VolSymDiff,
		AreaBound: result.AreaSlack}
	body.n.Payload = result
	return body, true, nil
}
