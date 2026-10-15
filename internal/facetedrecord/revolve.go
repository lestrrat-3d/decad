package facetedrecord

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

type RevolveSectorContact struct {
	Segment int
	Line    sketch.ConeLineCrossing
	Fit     sketch.ConeFitCrossing
}

type RevolveSectorBoundary struct {
	InsideSegment, OutsideWitness int
	Contacts                      [2]RevolveSectorContact
	FitRoute                      bool
	Sides                         []sketch.ConeCurveSide
	SourceRevision                uint64
}

// RevolveLoftSectorCertificate keeps the immutable source records and the
// Sketch answers that identify one connected exposed sector of a point loft.
// Mesh stations and their displacement remain a separate construction proof.
type RevolveLoftSectorCertificate struct {
	Blank         *topo.Body
	Section       RevolveSectionCertificate
	Trimmed       Payload
	Trim          *PointConeTrimRecord
	Base          Payload
	Boundary      RevolveSectorBoundary
	Sector        [2]float64
	Loop          []RevolveSectorNode
	CrossingBound float64
}

type RevolveSectorNode struct {
	Point        r3.Vec
	GroupToNext  int
	Crossing     int
	RootBoundary bool
	Angle        float64
}

// RevolveSectionSpan is one straight side of a recorded Revolve section.
// Several whole source lines may form the same side when they continue
// exactly forward on one line and Revolve retained one source face for them.
type RevolveSectionSpan struct {
	Start, End  sectionrecord.Point2
	SourceEdges []int
	Face        *topo.Face
	Origins     []provenance.Ref
}

// RevolveSectionCertificate describes the authenticated finite section and
// one candidate conical contact face. Its spans stay in source walk order.
type RevolveSectionCertificate struct {
	Blank              *topo.Body
	Spans              []RevolveSectionSpan
	Frame              r3.Frame
	AxisSpan, RootSpan int
	RootStart, RootEnd sectionrecord.Point2
	RootCone           surfacegeom.Cone
	ConeGap            float64
}

func (section RevolveSectionCertificate) ConeSection() sketch.ConeSection {
	angle, _ := section.RootCone.HalfAngle.In(units.Radian)
	return sketch.ConeSection{Apex: section.RootCone.Origin,
		Axis: section.RootCone.Axis, Slope: math.Tan(angle)}
}

func (section RevolveSectionCertificate) MeridianEquation() (*big.Rat, *big.Rat) {
	x0, x1 := proofarith.FloatRat(section.RootStart.U), proofarith.FloatRat(section.RootEnd.U)
	r0, r1 := proofarith.FloatRat(section.RootStart.V), proofarith.FloatRat(section.RootEnd.V)
	m := new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), new(big.Rat).Sub(x1, x0))
	b := new(big.Rat).Sub(r0, new(big.Rat).Mul(m, x0))
	return m, b
}

// RevolveLoftJoinRecord retains one proved section/sector source and the
// bounds published by its joined mesh. Later placed sectors and cap-ring
// rewrites can authenticate against the same immutable source.
type RevolveLoftJoinRecord struct {
	Certificate                           *RevolveLoftSectorCertificate
	Blank                                 *topo.Body
	First                                 Payload
	Placed                                []Payload
	CapRing                               *RevolvedSectionRing
	BoundaryBound, VolumeBound, AreaBound float64
}

type RevolveLoftSectorPlacement struct {
	Source *PointConeTrimRecord
	Motion r3.Transform
}

// RevolvedSectionRing shares every angular station across the section's
// recorded faces. An on-axis corner has one vertex rather than a full ring.
type RevolvedSectionRing struct {
	Angles []float64
	Rings  [][]int
	Groups []int
}

func (r RevolvedSectionRing) Vertex(span, station int) int {
	if len(r.Rings[span]) == 1 {
		return r.Rings[span][0]
	}
	return r.Rings[span][station]
}
