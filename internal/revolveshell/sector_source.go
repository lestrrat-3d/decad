package revolveshell

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// RecordedSection does not classify a gear profile. It only checks a
// full axial Revolve of a finite, closed whole-line walk and binds each
// geometric side to exactly one retained source face. It returns the spans,
// the index of the one span on the axis, and the Revolve frame.
func RecordedSection(blank *topo.Body) ([]facetedrecord.RevolveSectionSpan, int, r3.Frame, bool) {
	var empty r3.Frame
	if blank == nil {
		return nil, -1, empty, false
	}
	rp, ok := blank.Payload.(revolveplan.Payload)
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
	points, ends := make([]sectionrecord.Point2, len(segments)), make([]sectionrecord.Point2, len(segments))
	seen := make(map[sectionrecord.Point2]struct{}, len(segments))
	for i, segment := range segments {
		line, ok := segment.(sectionrecord.LineSeg)
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
	spans := make([]facetedrecord.RevolveSectionSpan, len(corners))
	axisSpan := -1
	usedFaces := make(map[*topo.Face]struct{}, len(corners))
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
		face, ok := SpanFace(blank, *span)
		if !ok {
			return nil, -1, empty, false
		}
		if _, duplicate := usedFaces[face]; duplicate {
			return nil, -1, empty, false
		}
		usedFaces[face] = struct{}{}
		span.Face, span.Origins = face, append([]provenance.Ref(nil), face.Origins...)
	}
	if axisSpan < 0 {
		return nil, -1, empty, false
	}
	return spans, axisSpan, rp.Frame, true
}

// SpanFace returns the one face of the Revolve blank whose origins are
// exactly the span's source-edge roles from the blank's own producer and
// whose surface and orientation are the wall the span sweeps.
func SpanFace(blank *topo.Body, span facetedrecord.RevolveSectionSpan) (*topo.Face, bool) {
	rp, ok := blank.Payload.(revolveplan.Payload)
	if !ok {
		return nil, false
	}
	dz, dr := span.End.U-span.Start.U, span.End.V-span.Start.V
	w := survey2d.SegmentWalk{StartU: span.Start.U, StartV: span.Start.V,
		EndU: span.End.U, EndV: span.End.V, TanInU: dz, TanInV: dr}
	kind := revolveaxis.WallCone
	if dz == 0 {
		kind = revolveaxis.WallPlane
	} else if dr == 0 {
		kind = revolveaxis.WallCylinder
	}
	surface, reversed, err := rp.WallSurface(rp.Basis(), w, kind)
	if err != nil {
		return nil, false
	}
	roles := make(map[string]struct{}, len(span.SourceEdges))
	for _, index := range span.SourceEdges {
		roles[fmt.Sprintf("side(0,%d)", index)] = struct{}{}
	}
	var matched *topo.Face
	for _, face := range topobuild.Faces(blank) {
		origins := face.Origins
		if len(origins) != len(roles) {
			continue
		}
		seen := make(map[string]struct{}, len(origins))
		valid := true
		for _, origin := range origins {
			if origin.Producer != blank.Origin.Producer {
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
			face.Surface == surface && face.Reversed == reversed {
			if matched != nil {
				return nil, false
			}
			matched = face
		}
	}
	return matched, matched != nil
}

// RootCandidates returns a certificate for every non-axis side of the
// recorded section whose face is a finite world-X cone with its apex on the
// axis and both span ends strictly past the apex.
func RootCandidates(blank *topo.Body) []facetedrecord.RevolveSectionCertificate {
	spans, axisSpan, frame, ok := RecordedSection(blank)
	if !ok {
		return nil
	}
	var candidates []facetedrecord.RevolveSectionCertificate
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
		candidates = append(candidates, facetedrecord.RevolveSectionCertificate{
			Blank: blank, Spans: spans, Frame: frame,
			AxisSpan: axisSpan, RootSpan: i, RootStart: start, RootEnd: end,
			RootCone: cone, ConeGap: gap})
	}
	return candidates
}

// CertifySector tries each authenticated finite conical face of
// the Revolve section. The source must certify exactly one face as the
// contact surface; a held geometric resemblance never selects one. The
// tooth's toe and heel limits and the blank must share one axis, checked
// before any boundary certificate is requested
// (agent-docs/revolve-axis-design.md §4.4).
func CertifySector(blank, sector *topo.Body) (*facetedrecord.RevolveLoftSectorCertificate, bool) {
	if sector == nil {
		return nil, false
	}
	fp, ok := sector.Payload.(facetedrecord.Payload)
	if !ok || fp.Xform != r3.Identity() || fp.PointCone == nil ||
		fp.PointCone.Lower == nil || fp.PointCone.Upper == nil ||
		fp.PointCone.Base.PointSection == nil ||
		fp.PointCone.Base.PointSection.Apex != (r3.Vec{}) ||
		fp.PointCone.Base.Xform != r3.Identity() {
		return nil, false
	}
	trim := fp.PointCone
	if !sectorAxesMatch(blank, trim) {
		return nil, false
	}
	if !facetedrecord.PointSourceCurrent(trim.Base.PointSection) {
		return nil, false
	}
	var selected *facetedrecord.RevolveLoftSectorCertificate
	for _, section := range RootCandidates(blank) {
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
		selected = &facetedrecord.RevolveLoftSectorCertificate{Blank: blank, Section: section,
			Trimmed: fp, Trim: trim, Base: trim.Base, Boundary: boundary}
	}
	return selected, selected != nil
}

// sectorAxesMatch reports whether both trim limits are about the blank's axis
// (coneaxis.OfRevolve), compared bitwise (agent-docs/revolve-axis-design.md
// §3.3). A canonical axis never equals a general one, so a +X blank with a
// tooth trimmed about another axis declines. Every Union entry reaches the
// sector route through CertifySector or through a join certificate that
// passed it.
func sectorAxesMatch(blank *topo.Body, trim *facetedrecord.PointConeTrimRecord) bool {
	if blank == nil {
		return false
	}
	rp, ok := blank.Payload.(revolveplan.Payload)
	if !ok {
		return false
	}
	axis, ok := coneaxis.OfRevolve(rp.Frame, rp.Ax, rp.AxisVec)
	return ok && trim.Lower.Axis == axis && trim.Upper.Axis == axis
}
