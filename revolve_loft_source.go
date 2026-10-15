package decad

import (
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// revolveSectionSpan is one straight side of a recorded Revolve section.
// Several whole source lines may form the same side when they continue
// exactly forward on one line and Revolve retained one source face for them.
type revolveSectionSpan struct {
	Start, End  planePoint
	SourceEdges []int
	Face        *Face
	Origins     []provenance.Ref
}

// revolveSectionCertificate describes the authenticated finite section and
// one candidate conical contact face. Its spans stay in source walk order.
type revolveSectionCertificate struct {
	Blank              *Body
	Spans              []revolveSectionSpan
	Frame              r3.Frame
	AxisSpan, RootSpan int
	RootStart, RootEnd planePoint
	RootCone           surfacegeom.Cone
	ConeGap            float64
}

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
		if !revolveForwardCollinear(previous, points[i], points[(i+1)%len(points)]) {
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
		span.Face, span.Origins = face, face.originRefs()
	}
	if axisSpan < 0 {
		return nil, -1, empty, false
	}
	return spans, axisSpan, rp.Frame, true
}

func revolveForwardCollinear(a, b, c planePoint) bool {
	ax := new(big.Rat).Sub(proofarith.FloatRat(b.U), proofarith.FloatRat(a.U))
	ay := new(big.Rat).Sub(proofarith.FloatRat(b.V), proofarith.FloatRat(a.V))
	bx := new(big.Rat).Sub(proofarith.FloatRat(c.U), proofarith.FloatRat(b.U))
	by := new(big.Rat).Sub(proofarith.FloatRat(c.V), proofarith.FloatRat(b.V))
	cross := new(big.Rat).Sub(new(big.Rat).Mul(ax, by), new(big.Rat).Mul(ay, bx))
	dot := new(big.Rat).Add(new(big.Rat).Mul(ax, bx), new(big.Rat).Mul(ay, by))
	return cross.Sign() == 0 && dot.Sign() > 0
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
		cone, ok := span.Face.n.Surface.(surfacegeom.Cone)
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
		gap := max(revolveMeridianResidual(start, slope, cone.Origin.X),
			revolveMeridianResidual(end, slope, cone.Origin.X))
		if proofbound.IsNonFinite(gap) {
			continue
		}
		candidates = append(candidates, revolveSectionCertificate{
			Blank: blank, Spans: spans, Frame: frame,
			AxisSpan: axisSpan, RootSpan: i, RootStart: start, RootEnd: end,
			RootCone: cone, ConeGap: gap})
	}
	return candidates
}

func revolveMeridianResidual(point planePoint, slope, originX float64) float64 {
	r := new(big.Rat).Sub(proofarith.FloatRat(point.V),
		new(big.Rat).Mul(proofarith.FloatRat(slope),
			new(big.Rat).Sub(proofarith.FloatRat(point.U), proofarith.FloatRat(originX))))
	if r.Sign() < 0 {
		r.Neg(r)
	}
	return proofbound.RatFloatUp(r)
}

func (section revolveSectionCertificate) ConeSection() sketch.ConeSection {
	angle, _ := section.RootCone.HalfAngle.In(units.Radian)
	return sketch.ConeSection{Apex: section.RootCone.Origin,
		Axis: section.RootCone.Axis, Slope: math.Tan(angle)}
}

func (section revolveSectionCertificate) MeridianEquation() (*big.Rat, *big.Rat) {
	x0, x1 := proofarith.FloatRat(section.RootStart.U), proofarith.FloatRat(section.RootEnd.U)
	r0, r1 := proofarith.FloatRat(section.RootStart.V), proofarith.FloatRat(section.RootEnd.V)
	m := new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), new(big.Rat).Sub(x1, x0))
	b := new(big.Rat).Sub(r0, new(big.Rat).Mul(m, x0))
	return m, b
}
