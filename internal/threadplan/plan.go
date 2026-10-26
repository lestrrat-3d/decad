// Package threadplan computes the groove profile and coil length of an ISO
// metric thread (agent-docs/thread-design.md). It works on plain millimetre
// data and touches no body, document or sketch.
package threadplan

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// ISO 68-1 basic profile ratios, as fractions of the pitch P. H is the height
// of the sharp 60 degree triangle, √3/2·P.
const (
	height          = 0.8660254037844386   // √3/2
	externalDepth   = height * 17.0 / 24.0 // h3 of an external thread, 0.61343·P
	internalDepth   = height * 5.0 / 8.0   // depth of an internal thread's groove, 0.54127·P
	externalFlat    = 1.0 / 8.0            // root flat of an external thread
	internalFlat    = 1.0 / 4.0            // root flat of an internal thread
	flankSlope      = 0.5773502691896257   // tan 30 degrees: each flank widens the groove by this per unit of depth
	mouthOvershoot  = 0.05                 // how far the groove mouth opens past the surface, as a fraction of P
	startMargin     = 0.05                 // how far the first groove lies clear of the start plane, as a fraction of P
	maxProfileRatio = 0.95                 // the groove's axial width stays below this fraction of P, so turns never meet
)

// Plan is the groove profile and the coil that cuts it.
type Plan struct {
	// Profile is the groove polygon in the axis plane, as (radial, axial)
	// millimetres, its corners in order around the boundary. The axial coordinate is measured
	// from the start of the threaded length along the axis, and the first
	// groove is centred just before that start.
	Profile [4][2]float64
	// Turns is the number of turns, fractional, that carries the groove past
	// the far end of the threaded length.
	Turns float64
}

// Compute plans a thread cut into a cylindrical surface of the given radius.
// An external thread cuts a groove into a rod, and radius is its major radius.
// An internal thread cuts a groove out of a bore, and radius is its minor
// (drilled) radius. length is the axial length of the threaded face. All three
// numbers are positive and finite.
func Compute(radius, pitch, length float64, external bool) (Plan, error) {
	for name, v := range map[string]float64{"radius": radius, "pitch": pitch, "length": length} {
		if !(v > 0) || math.IsInf(v, 0) {
			return Plan{}, fmt.Errorf(`%w: the thread %s must be positive and finite`, decaderr.ErrDegenerate, name)
		}
	}
	depth, flat := externalDepth*pitch, externalFlat*pitch
	root, mouth := radius-depth, radius+mouthOvershoot*pitch
	if !external {
		depth, flat = internalDepth*pitch, internalFlat*pitch
		root, mouth = radius+depth, radius-mouthOvershoot*pitch
	}
	if root <= 0 || mouth <= 0 {
		return Plan{}, fmt.Errorf(`%w: a %g mm pitch is too coarse for a %g mm radius: the groove reaches the axis`,
			decaderr.ErrDegenerate, pitch, radius)
	}
	half := flat/2 + math.Abs(mouth-root)*flankSlope
	if 2*half >= maxProfileRatio*pitch {
		return Plan{}, fmt.Errorf(`%w: the groove is %g mm wide, too wide for a %g mm pitch`,
			decaderr.ErrDegenerate, 2*half, pitch)
	}
	first := -half - startMargin*pitch
	var p [4][2]float64
	if external {
		p = [4][2]float64{{root, first - flat/2}, {root, first + flat/2}, {mouth, first + half}, {mouth, first - half}}
	} else {
		p = [4][2]float64{{mouth, first - half}, {root, first - flat/2}, {root, first + flat/2}, {mouth, first + half}}
	}
	turns := (length + 2*half + 2*startMargin*pitch) / pitch
	return Plan{Profile: p, Turns: turns}, nil
}
