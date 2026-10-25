package revolveshell

import (
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
