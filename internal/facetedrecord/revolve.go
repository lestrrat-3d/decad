package facetedrecord

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
//
// Axis is the blank's axis value (coneaxis.OfRevolve). General is nil exactly
// when the blank passed the frame predicate (coneaxis.FrameCanonical); every
// reading of RootStart, RootEnd, RootCone and ConeGap then has its +X meaning.
// A non-nil General carries the exact A-scaled readings of the root span and
// the reported cone (agent-docs/revolve-axis-design.md §4.1, §4.2), and its
// presence, not the axis value, selects the general arithmetic: RootStart and
// RootEnd stay plane-local points, ordered by the axial coordinate p·A.
type RevolveSectionCertificate struct {
	Blank              *topo.Body
	Spans              []RevolveSectionSpan
	Frame              r3.Frame
	AxisSpan, RootSpan int
	RootStart, RootEnd sectionrecord.Point2
	RootCone           surfacegeom.Cone
	ConeGap            float64
	Axis               coneaxis.Axis
	General            *RevolveSectionAxis
}

// RevolveSectionAxis holds the exact readings of a root span about a general
// axis A (agent-docs/revolve-axis-design.md §4.1, §4.2, §5.2). Every scalar is
// A-scaled: a world point p has the axial coordinate Z = p·A and the radial
// coordinate R = |p×A|.
type RevolveSectionAxis struct {
	// Cone is the reported cone {float(P*), A, float(k*)}. Sketch certifies
	// against exactly this value.
	Cone sketch.ConeSection
	// Slope is the exact root slope k* = (R1−R0)/(Z1−Z0) and Apex the exact
	// apex scalar Z_a = Z0 − R0/k*. The root meridian is R = k*·(Z − Z_a).
	Slope, Apex *big.Rat
	// RootZ and RootR are the exact Z and R of RootStart and RootEnd.
	RootZ, RootR [2]*big.Rat
	// ApexGap is δ_a ≥ |Cone.Apex − P*| and SlopeGap ≥ |k* − Cone.Slope|.
	ApexGap, SlopeGap float64
	// W is the blank basis's held axis direction, and E0 and E1 are its
	// angle-0 and angle-π/2 directions (revolvemesh.RevolveBasis, §3.7).
	// massmoment.FrameTilt charges (W, E0, E1) against an exact frame about
	// the axis (§5.6).
	W, E0, E1 r3.Vec
}

// ConeGap returns δ_a + |k*−s|·(reach + δ_a) rounded upward, the distance
// bound of §5.2 between the true root cone and the reported one over the
// points whose axial distance from P* is at most reach millimetres. The
// translation by Cone.Apex − P* moves each point by at most δ_a and shifts
// its axial distance by at most δ_a, so a slope gap read at the translated
// point takes reach + δ_a.
func (g *RevolveSectionAxis) ConeGap(reach float64) float64 {
	return proofbound.AbsSumUpper(g.ApexGap,
		proofbound.ProductUpper(g.SlopeGap, proofbound.AbsSumUpper(reach, g.ApexGap)))
}

// ConeSection returns the cone Sketch certifies against: the stored general
// cone, or for a +X certificate the cone read from the root face's tag.
func (section RevolveSectionCertificate) ConeSection() sketch.ConeSection {
	if section.General != nil {
		return section.General.Cone
	}
	angle, _ := section.RootCone.HalfAngle.In(units.Radian)
	return sketch.ConeSection{Apex: section.RootCone.Origin,
		Axis: section.RootCone.Axis, Slope: math.Tan(angle)}
}

// MeridianEquation returns the +X root meridian r = m·x + b read from the
// plane-local root ends. A general certificate reads General instead.
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
