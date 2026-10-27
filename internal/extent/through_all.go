package extent

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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

// denotedCaps is what a through-all stop reads of a stop body whose material
// lies between two planar caps of one record: the record's frame and
// placement, both held cap levels, and the exact level each denotes, nil
// where none is recorded (agent-docs/stacked-prism-design.md §2.1).
type denotedCaps interface {
	DenotedCapLevels() (frame r3.Frame, xform r3.Transform, z0, z1 float64, d0, d1 *big.Rat)
}

// farCapDenotation is the exact sweep-frame level of a stop body's far cap
// along travel, or nil when the body states no such cap or the cap records no
// denoted level. Both caps must be exact level sets of the sweep frame
// (PlaneStop), so the body's material ends at the far one, and the sweep
// normal must be exactly a unit vector, so a far coordinate along the normal
// is that level. The far cap is the one whose held level lies strictly
// farther along travel.
func farCapDenotation(b *topo.Body, sweep r3.Frame, travel float64) *big.Rat {
	dc, ok := b.Payload.(denotedCaps)
	if !ok {
		return nil
	}
	n := ratVec(sweep.N())
	if n[0] == nil || n[1] == nil || n[2] == nil || ratDot(n, n).Cmp(big.NewRat(1, 1)) != 0 {
		return nil
	}
	frame, xform, z0, z1, d0, d1 := dc.DenotedCapLevels()
	c0, _, ok0 := PlaneStop(frame, xform, z0, sweep)
	c1, _, ok1 := PlaneStop(frame, xform, z1, sweep)
	t := proofarith.FloatRat(travel)
	if !ok0 || !ok1 || t == nil {
		return nil
	}
	order := new(big.Rat).Mul(t, new(big.Rat).Sub(c1, c0)).Sign()
	far := d1
	switch {
	case order == 0:
		return nil
	case order < 0:
		far = d0
	}
	if far == nil {
		return nil
	}
	c, _, ok := PlaneStopAt(frame, xform, far, sweep)
	if !ok {
		return nil
	}
	return c
}

// ThroughAll resolves a through-all stop over the live bodies: the sweep runs
// from the sketch plane frame in the travel sense through the far side of
// every body it meets (agent-docs/evaluator-design.md §5). It returns the signed
// stop coordinate along the plane normal, that coordinate's proven axial
// displacement, the met bodies' producers, nearest far side first, and the
// exact level the stop denotes, nil when no record proves one (ThroughStops.Finish).
// No body in the path is ErrDegenerate.
func ThroughAll(frame r3.Frame, travel float64, live []*topo.Body) (float64, float64, []provenance.Producer, *big.Rat, error) {
	dir := frame.N().Scale(travel)
	stops := NewThroughStops[provenance.Producer](frame.Origin(), dir, travel)
	for _, b := range live {
		// A sheet encloses no material to stop a sweep.
		if b.Kind == topo.BodySheet {
			continue
		}
		ext, ok := b.Payload.(directionalExtent)
		if !ok {
			return 0, 0, nil, nil, fmt.Errorf(`%w: a through-all stop needs every live body's directional extent, and this evaluator did not build one of them`, decaderr.ErrUnsupported)
		}
		_, hi, bound, err := ext.ExtentAlong(dir)
		if err != nil {
			return 0, 0, nil, nil, err
		}
		if err := stops.Add(hi, bound, payloadAxialDelta(b), b.Origin.Producer, farCapDenotation(b, frame, travel)); err != nil {
			return 0, 0, nil, nil, err
		}
	}
	return stops.Finish()
}
