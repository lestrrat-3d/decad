package revolveshell

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// SectorUnionMany admits one Revolve and one authentic point
// Loft source, followed by copies of that same source. Operand order does
// not assign a geometric role; source records do. The certificate may be
// about +X or about a general axis (agent-docs/revolve-axis-design.md).
func SectorUnionMany(ctx context.Context, env topo.Env,
	operands []*topo.Body) (*topo.Body, bool, error) {
	for blankIndex, blank := range operands {
		for firstIndex, first := range operands {
			if firstIndex == blankIndex {
				continue
			}
			cert, ok := CertifySector(blank, first)
			if !ok {
				continue
			}
			if err := TraceSectorLoop(cert); err != nil {
				return nil, true, err
			}
			sources := make([]facetedrecord.Payload, 0, len(operands)-1)
			sources = append(sources, cert.Trimmed)
			for i, operand := range operands {
				if i == blankIndex || i == firstIndex {
					continue
				}
				placed, ok := operand.Payload.(facetedrecord.Payload)
				if !ok || placed.RevolveLoftSector == nil ||
					placed.RevolveLoftSector.Source != cert.Trim ||
					placed.Xform != placed.RevolveLoftSector.Motion {
					return nil, true, fmt.Errorf("%w: placed sector lost its source", decaderr.ErrUnsupported)
				}
				sources = append(sources, placed)
			}
			if !RevolveSectorSourcesSeparate(sources, cert.Section.Axis) {
				return nil, true, fmt.Errorf("%w: complete sector sources are not separate", decaderr.ErrUnsupported)
			}
			body, err := BuildSectorsJoin(ctx, env,
				cert, sources)
			return body, true, err
		}
	}
	return nil, false, nil
}

// SectorUnion authenticates source geometry before construction. The
// certificate may be about +X or about a general axis.
func SectorUnion(ctx context.Context, op meshbool.OperationKind,
	env topo.Env, a, b *topo.Body) (*topo.Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	blank, sector := a, b
	cert, ok := CertifySector(blank, sector)
	if !ok {
		blank, sector = b, a
		cert, ok = CertifySector(blank, sector)
		if !ok {
			return nil, false, nil
		}
	}
	if err := TraceSectorLoop(cert); err != nil {
		return nil, true, err
	}
	body, err := BuildSectorsJoin(ctx, env, cert,
		[]facetedrecord.Payload{cert.Trimmed})
	return body, true, err
}

// NextSectorUnion rebuilds a serial join from the retained
// authenticated Revolve and point-Loft sources. The faceted joined operand
// is not used as a geometric source for the next contact. The new copy's
// motion must be a rotation about the certificate's axis that
// massmoment.AxisRotation admits: a reflected, translated or obtuse copy, or
// one whose axis drifts past massmoment.DriftSinLimit, declines.
func NextSectorUnion(ctx context.Context, op meshbool.OperationKind,
	env topo.Env, a, b *topo.Body) (*topo.Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	joined, placed := a, b
	jp, ok := joined.Payload.(facetedrecord.Payload)
	if !ok || jp.RevolveLoftJoin == nil {
		joined, placed = b, a
		jp, ok = joined.Payload.(facetedrecord.Payload)
	}
	p, placedOK := placed.Payload.(facetedrecord.Payload)
	if !ok || !placedOK || jp.RevolveLoftJoin == nil ||
		jp.Xform != r3.Identity() || p.RevolveLoftSector == nil ||
		p.RevolveLoftSector.Source == nil ||
		jp.RevolveLoftJoin.Certificate == nil ||
		jp.RevolveLoftJoin.Certificate.Trim != p.RevolveLoftSector.Source ||
		p.Xform != p.RevolveLoftSector.Motion {
		return nil, false, nil
	}
	axis := jp.RevolveLoftJoin.Certificate.Section.Axis
	if _, _, ok := massmoment.AxisRotation(p.RevolveLoftSector.Motion, axis); !ok {
		return nil, false, nil
	}
	record := jp.RevolveLoftJoin
	sectors := make([]facetedrecord.Payload, 0, len(record.Placed)+2)
	sectors = append(sectors, record.First)
	sectors = append(sectors, record.Placed...)
	sectors = append(sectors, p)
	if !RevolveSectorSourcesSeparate(sectors, axis) {
		return nil, false, nil
	}
	body, err := BuildSectorsJoin(ctx, env,
		record.Certificate, sectors)
	return body, true, err
}
