package revolveshell

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// RingVertexAbout returns the vertex a general certificate's section ring
// holds for plane point p at angle, and the SweptPointGap it charges for it.
// It reports false when the certificate has lost its Revolve record or the
// angle has no trigonometric enclosure.
func RingVertexAbout(section facetedrecord.RevolveSectionCertificate, p sectionrecord.Point2, onAxis bool,
	angle float64) (r3.Vec, float64, bool) {
	lift, ok := newRingLiftAbout(section)
	if !ok {
		return r3.Vec{}, 0, false
	}
	sin, cos, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
	if !ok {
		return r3.Vec{}, 0, false
	}
	held, gap := lift.vertex(p, onAxis, angle, sin, cos)
	return held, gap, true
}

// BoreRingAbout is the general bore ring of a certificate from the general
// record branch: the inner ring vertices on the caps whose plane points are
// low and high, the bound the annular Cut charges for them with the tool
// deviation deviation, and the plane point and per-coordinate bound that
// SweptPointGap reads on the low cap.
func BoreRingAbout(ctx context.Context, section facetedrecord.RevolveSectionCertificate, low, high sectionrecord.Point2,
	radius float64, angles []float64, maxStep, deviation float64) ([]r3.Vec, []r3.Vec, float64,
	sectionrecord.Point2, proofbound.WalkEndBound, error) {
	lift, ok := newRingLiftAbout(section)
	if !ok {
		return nil, nil, 0, sectionrecord.Point2{}, proofbound.WalkEndBound{}, errNoLift
	}
	lowZ, _ := planeScaledRat(section.Axis, section.Frame, low)
	highZ, _ := planeScaledRat(section.Axis, section.Frame, high)
	bore := generalBoreAxis{axis: section.Axis, frame: section.Frame, lift: lift, lowPoint: low, highPoint: high,
		lowZ: lowZ, highZ: highZ, deviation: deviation}
	plane, within, ok := bore.borePlanePoint(lowZ, radius)
	if !ok {
		return nil, nil, 0, sectionrecord.Point2{}, proofbound.WalkEndBound{}, errNoLift
	}
	lowRing, highRing, bound, err := bore.ring(ctx, SectorBoreCaps{}, radius, angles, maxStep)
	return lowRing, highRing, bound, plane, within, err
}

var errNoLift = errors.New("certificate has no general lift")

// Bore tool readings about a general axis (agent-docs/revolve-axis-design.md
// §9.1, §9.4).
var (
	CoaxialBoreFrame  = coaxialBoreFrame
	BoreToolSpansCaps = boreToolSpansCaps
	BoreToolDeviation = boreToolDeviation
)

// BoreBandAbout is the general band reading of a certificate from the
// general record branch whose section ring holds verts: the band before
// the low cap, crossed by a bore of the given radius whose tool deviates by
// deviation. It returns the clipped ring, the high inner ring and the bore
// bound.
func BoreBandAbout(ctx context.Context, section facetedrecord.RevolveSectionCertificate, verts []r3.Vec,
	ring facetedrecord.RevolvedSectionRing, radius, maxStep, deviation float64) ([]r3.Vec, []r3.Vec, float64, error) {
	lift, ok := newRingLiftAbout(section)
	if !ok {
		return nil, nil, 0, errNoLift
	}
	count := len(section.Spans)
	low, high := (section.AxisSpan+count-1)%count, (section.AxisSpan+1)%count
	band := (low + count - 1) % count
	lowZ, _ := planeScaledRat(section.Axis, section.Frame, section.Spans[low].Start)
	highZ, _ := planeScaledRat(section.Axis, section.Frame, section.Spans[high].End)
	bore := generalBoreAxis{axis: section.Axis, frame: section.Frame, lift: lift,
		lowPoint: section.Spans[low].Start, highPoint: section.Spans[high].End,
		lowZ: lowZ, highZ: highZ, deviation: deviation}
	caps := SectorBoreCaps{Low: low, High: high, LowOuter: ring.Rings[low], HighOuter: ring.Rings[(high+1)%count]}
	joined := facetedrecord.Payload{Verts: verts,
		RevolveLoftJoin: &facetedrecord.RevolveLoftJoinRecord{CapRing: &ring}}
	cut, err := bore.band(ctx, sectorBandInput{joined: joined, caps: caps, band: section.Spans[band],
		rootRing: ring.Rings[band], radius: radius, maxStep: maxStep})
	return cut.cut, cut.high, cut.boreBound, err
}
