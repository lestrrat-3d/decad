package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// recordedRevolveSection does not classify a gear profile. It only checks a
// full axial Revolve of a finite, closed whole-line walk and binds each
// geometric side to exactly one retained source face.
func recordedRevolveSection(blank *Body) ([]revolveSectionSpan, int, r3.Frame, bool) {
	var empty r3.Frame
	if blank == nil {
		return nil, -1, empty, false
	}
	rp, ok := blank.n.Payload.(revolvePayload)
	if !ok || !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 ||
		rp.Xform != r3.Identity() || len(rp.Profile.Holes) != 0 ||
		rp.Frame.Origin() != (r3.Vec{}) || rp.Frame.U() != (r3.Vec{X: 1}) ||
		rp.Frame.V() != (r3.Vec{Y: 1}) ||
		rp.Ax.AU != 0 || rp.Ax.AV != 0 || rp.Ax.DU != 1 || rp.Ax.DV != 0 ||
		rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 ||
		rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 {
		return nil, -1, empty, false
	}
	segments := rp.Profile.Outer.Segments
	if len(segments) < 4 || len(segments) > 128 {
		return nil, -1, empty, false
	}
	points, ends := make([]planePoint, len(segments)), make([]planePoint, len(segments))
	seen := make(map[planePoint]struct{}, len(segments))
	for i, segment := range segments {
		line, ok := segment.(lineSeg)
		if !ok || !((line.TStart == 0 && line.TEnd == 1) ||
			(line.TStart == 1 && line.TEnd == 0)) || line.Start == line.End ||
			!proofbound.FiniteVec(r3.Vec{X: line.Start.U, Y: line.Start.V}) ||
			!proofbound.FiniteVec(r3.Vec{X: line.End.U, Y: line.End.V}) {
			return nil, -1, empty, false
		}
		points[i], ends[i] = line.Start, line.End
		if line.TStart == 1 {
			points[i], ends[i] = ends[i], points[i]
		}
		if _, duplicate := seen[points[i]]; duplicate {
			return nil, -1, empty, false
		}
		seen[points[i]] = struct{}{}
	}
	for i := range ends {
		if ends[i] != points[(i+1)%len(points)] {
			return nil, -1, empty, false
		}
	}
	var corners []int
	for i := range points {
		previous := points[(i+len(points)-1)%len(points)]
		if !revolveproof.RevolveForwardCollinear(previous, points[i], points[(i+1)%len(points)]) {
			corners = append(corners, i)
		}
	}
	if len(corners) < 4 {
		return nil, -1, empty, false
	}
	spans := make([]revolveSectionSpan, len(corners))
	axisSpan := -1
	usedFaces := make(map[*Face]struct{}, len(corners))
	for j, start := range corners {
		end := corners[(j+1)%len(corners)]
		span := &spans[j]
		span.Start, span.End = points[start], points[end]
		for i := start; i != end; i = (i + 1) % len(points) {
			span.SourceEdges = append(span.SourceEdges, i)
		}
		if span.Start.U <= 0 || span.End.U <= 0 ||
			span.Start.V < 0 || span.End.V < 0 {
			return nil, -1, empty, false
		}
		if span.Start.V == 0 && span.End.V == 0 {
			if axisSpan >= 0 {
				return nil, -1, empty, false
			}
			axisSpan = j
			continue
		}
		face, ok := revolveSpanFace(blank, *span)
		if !ok {
			return nil, -1, empty, false
		}
		if _, duplicate := usedFaces[face]; duplicate {
			return nil, -1, empty, false
		}
		usedFaces[face] = struct{}{}
		span.Face, span.Origins = faceNode(face), face.originRefs()
	}
	if axisSpan < 0 {
		return nil, -1, empty, false
	}
	return spans, axisSpan, rp.Frame, true
}

func revolveSpanFace(blank *Body, span revolveSectionSpan) (*Face, bool) {
	rp, ok := blank.n.Payload.(revolvePayload)
	if !ok {
		return nil, false
	}
	dz, dr := span.End.U-span.Start.U, span.End.V-span.Start.V
	w := survey2d.SegmentWalk{StartU: span.Start.U, StartV: span.Start.V,
		EndU: span.End.U, EndV: span.End.V, TanInU: dz, TanInV: dr}
	kind := wallCone
	if dz == 0 {
		kind = wallPlane
	} else if dr == 0 {
		kind = wallCylinder
	}
	surface, reversed, err := rp.WallSurface(rp.Basis(), w, kind)
	if err != nil {
		return nil, false
	}
	roles := make(map[string]struct{}, len(span.SourceEdges))
	for _, index := range span.SourceEdges {
		roles[fmt.Sprintf("side(0,%d)", index)] = struct{}{}
	}
	var matched *Face
	for _, face := range blank.Faces() {
		origins := face.originRefs()
		if len(origins) != len(roles) {
			continue
		}
		seen := make(map[string]struct{}, len(origins))
		valid := true
		for _, origin := range origins {
			if origin.Producer != blank.n.Origin.Producer {
				valid = false
				break
			}
			if _, ok := roles[origin.Role]; !ok {
				valid = false
				break
			}
			seen[origin.Role] = struct{}{}
		}
		if valid && len(seen) == len(roles) &&
			face.n.Surface == surface && face.n.Reversed == reversed {
			if matched != nil {
				return nil, false
			}
			matched = face
		}
	}
	return matched, matched != nil
}

func revolveRootCandidates(blank *Body) []revolveSectionCertificate {
	spans, axisSpan, frame, ok := recordedRevolveSection(blank)
	if !ok {
		return nil
	}
	var candidates []revolveSectionCertificate
	for i, span := range spans {
		if span.Face == nil || span.Start.V == 0 || span.End.V == 0 {
			continue
		}
		cone, ok := span.Face.Surface.(surfacegeom.Cone)
		if !ok || cone.Axis != (r3.Vec{X: 1}) ||
			cone.Origin.Y != 0 || cone.Origin.Z != 0 ||
			!proofbound.FiniteVec(cone.Origin) {
			continue
		}
		radius, radiusErr := cone.Radius.In(units.Millimeter)
		angle, angleErr := cone.HalfAngle.In(units.Radian)
		if radiusErr != nil || angleErr != nil || radius != 0 ||
			!proofbound.FiniteVec(r3.Vec{X: angle}) || angle <= 0 || angle >= math.Pi/2 {
			continue
		}
		slope := math.Tan(angle)
		if !proofbound.FiniteVec(r3.Vec{X: slope}) || slope <= 0 {
			continue
		}
		start, end := span.Start, span.End
		if start.U > end.U {
			start, end = end, start
		}
		if start.U == end.U || start.U <= cone.Origin.X {
			continue
		}
		gap := max(revolveproof.RevolveMeridianResidual(start, slope, cone.Origin.X),
			revolveproof.RevolveMeridianResidual(end, slope, cone.Origin.X))
		if proofbound.IsNonFinite(gap) {
			continue
		}
		candidates = append(candidates, revolveSectionCertificate{
			Blank: bodyNode(blank), Spans: spans, Frame: frame,
			AxisSpan: axisSpan, RootSpan: i, RootStart: start, RootEnd: end,
			RootCone: cone, ConeGap: gap})
	}
	return candidates
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
	cert, ok := certifyRevolveLoftSector(blank, sector)
	if !ok {
		blank, sector = b, a
		cert, ok = certifyRevolveLoftSector(blank, sector)
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

// certifyRevolveLoftSector tries each authenticated finite conical face of
// the Revolve section. The source must certify exactly one face as the
// contact surface; a held geometric resemblance never selects one.
func certifyRevolveLoftSector(blank, sector *Body) (*revolveLoftSectorCertificate, bool) {
	if sector == nil {
		return nil, false
	}
	fp, ok := sector.n.Payload.(facetedPayload)
	if !ok || fp.Xform != r3.Identity() || fp.PointCone == nil ||
		fp.PointCone.Lower == nil || fp.PointCone.Upper == nil ||
		fp.PointCone.Base.PointSection == nil ||
		fp.PointCone.Base.PointSection.Apex != (r3.Vec{}) ||
		fp.PointCone.Base.Xform != r3.Identity() {
		return nil, false
	}
	trim := fp.PointCone
	if !facetedrecord.PointSourceCurrent(trim.Base.PointSection) {
		return nil, false
	}
	var selected *revolveLoftSectorCertificate
	for _, section := range revolveRootCandidates(blank) {
		boundary, ok := revolveproof.CertifyRevolveConnectorBoundary(trim.Base.PointSection,
			section, trim.Base)
		if !ok {
			boundary, ok = revolveproof.CertifyRevolveFitBoundary(trim.Base.PointSection,
				section, trim.Base)
			if !ok {
				continue
			}
		}
		if selected != nil {
			return nil, false
		}
		selected = &revolveLoftSectorCertificate{Blank: bodyNode(blank), Section: section,
			Trimmed: fp, Trim: trim, Base: trim.Base, Boundary: boundary}
	}
	return selected, selected != nil
}
