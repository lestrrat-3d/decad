package extent

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// directionalExtent is what a through-all stop needs from a body's payload:
// the body's extent interval along an arbitrary world direction, BESIDE the
// proven displacement its two endpoints carry (agent-docs/spline-design.md
// §6.2/§6.4). A payload that cannot state the interval returns ErrUnsupported.
type directionalExtent interface {
	ExtentAlong(g r3.Vec) (float64, float64, float64, error)
}

// axialDisplacement is what a body-relative stop reads of a stop body's
// payload: a proven displacement covering its analytic planar stop faces. A
// level derived from such a face inherits it, so a stop cannot launder a held
// coordinate back into an exact one.
type axialDisplacement interface {
	AxialDelta() float64
}

// payloadAxialDelta returns the payload's stop-face displacement, 0 for a
// payload that states none. The figure a linear stop publishes composes this
// term with its own arithmetic.
func payloadAxialDelta(b *topo.Body) float64 {
	ad, ok := b.Payload.(axialDisplacement)
	if !ok {
		return 0
	}
	return ad.AxialDelta()
}

// ThroughAll resolves a through-all stop over the live bodies: the sweep runs
// from the sketch plane frame in the travel sense through the far side of
// every body it meets (agent-docs/evaluator-design.md §5). It returns the signed
// stop coordinate along the plane normal, that coordinate's proven axial
// displacement and the met bodies' producers, nearest far side first. No body
// in the path is ErrDegenerate.
func ThroughAll(frame r3.Frame, travel float64, live []*topo.Body) (float64, float64, []provenance.Producer, error) {
	dir := frame.N().Scale(travel)
	stops := NewThroughStops[provenance.Producer](frame.Origin(), dir, travel)
	for _, b := range live {
		// A sheet encloses no material to stop a sweep.
		if b.Kind == topo.BodySheet {
			continue
		}
		ext, ok := b.Payload.(directionalExtent)
		if !ok {
			return 0, 0, nil, fmt.Errorf(`%w: a through-all stop needs every live body's directional extent, and this evaluator did not build one of them`, decaderr.ErrUnsupported)
		}
		_, hi, bound, err := ext.ExtentAlong(dir)
		if err != nil {
			return 0, 0, nil, err
		}
		if err := stops.Add(hi, bound, payloadAxialDelta(b), b.Origin.Producer); err != nil {
			return 0, 0, nil, err
		}
	}
	return stops.Finish()
}
