package capedge

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ConvexVertices reads a whole, closed, strictly convex straight section.
// It returns vertices in recorded walk order so a caller can locate its edge.
func ConvexVertices(profile momentinput.Profile) ([]sectionrecord.Point2, bool) {
	segments := profile.Outer.Segments
	if len(segments) < 3 {
		return nil, false
	}
	lines := make([]sectionrecord.LineSeg, len(segments))
	vertices := make([]sectionrecord.Point2, len(segments))
	for i, segment := range segments {
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok || !((line.TStart == 0 && line.TEnd == 1) || (line.TStart == 1 && line.TEnd == 0)) {
			return nil, false
		}
		lines[i] = line
		vertices[i] = line.Start
	}
	for i, line := range lines {
		if line.End != vertices[(i+1)%len(vertices)] {
			return nil, false
		}
	}
	var turnSign int
	for i := range vertices {
		a, b, c := vertices[i], vertices[(i+1)%len(vertices)], vertices[(i+2)%len(vertices)]
		abU := new(big.Rat).Sub(proofarith.FloatRat(b.U), proofarith.FloatRat(a.U))
		abV := new(big.Rat).Sub(proofarith.FloatRat(b.V), proofarith.FloatRat(a.V))
		bcU := new(big.Rat).Sub(proofarith.FloatRat(c.U), proofarith.FloatRat(b.U))
		bcV := new(big.Rat).Sub(proofarith.FloatRat(c.V), proofarith.FloatRat(b.V))
		turn := new(big.Rat).Sub(new(big.Rat).Mul(abU, bcV), new(big.Rat).Mul(abV, bcU)).Sign()
		if turn == 0 || (turnSign != 0 && turn != turnSign) {
			return nil, false
		}
		turnSign = turn
	}
	return vertices, true
}

// Eligible checks the selected edge's length, terminal directions, and
// clearance from unrelated section vertices against the cutter radius.
func Eligible(vertices []sectionrecord.Point2, match int, radius float64) bool {
	if len(vertices) < 3 || match < 0 || match >= len(vertices) {
		return false
	}
	a, b := vertices[match], vertices[(match+1)%len(vertices)]
	du, dv := b.U-a.U, b.V-a.V
	prev := vertices[(match+len(vertices)-1)%len(vertices)]
	next := vertices[(match+2)%len(vertices)]
	if (du == 0) != (dv == 0) &&
		du*(a.U-prev.U)+dv*(a.V-prev.V) == 0 &&
		du*(next.U-b.U)+dv*(next.V-b.V) == 0 {
		return false
	}
	for _, neighbour := range [2]sectionrecord.Point2{{U: a.U - prev.U, V: a.V - prev.V},
		{U: next.U - b.U, V: next.V - b.V}} {
		parallel := math.Abs(du*neighbour.U + dv*neighbour.V)
		cross := math.Abs(du*neighbour.V - dv*neighbour.U)
		if cross == 0 || parallel >= 4*cross {
			return false
		}
	}
	length := math.Hypot(b.U-a.U, b.V-a.V)
	if length <= 4*radius {
		return false
	}
	for i, v := range vertices {
		if i == match || i == (match+1)%len(vertices) {
			continue
		}
		distance := math.Abs((b.U-a.U)*(v.V-a.V)-(b.V-a.V)*(v.U-a.U)) / length
		if distance <= 4*radius || proofbound.IsNonFinite(distance) {
			return false
		}
	}
	return true
}

// QuarterCutter is the part outside a radius-r quarter disk at the
// intersection of two outward coordinate half-planes. Its straight sides
// extend beyond both faces to give the boolean crossing edges.
func QuarterCutter(r float64) momentinput.Profile {
	p0, p1, p2 := sectionrecord.Point2{U: -r, V: 0}, sectionrecord.Point2{U: -r, V: r}, sectionrecord.Point2{U: r, V: r}
	p3, p4, center := sectionrecord.Point2{U: r, V: -r}, sectionrecord.Point2{U: 0, V: -r}, sectionrecord.Point2{U: -r, V: -r}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: center, Start: p4, End: p0, TStart: 1, TEnd: 0},
		sectionrecord.LineSeg{Start: p4, End: p3, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p3, End: p2, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p2, End: p1, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p1, End: p0, TStart: 0, TEnd: 1},
	}}}
}
