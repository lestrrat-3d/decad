package sweeprun

import (
	"errors"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/seal"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"
)

// errSweepPoseBudget is how a producer reports an exhausted pose budget.
var errSweepPoseBudget = errors.New("decad: sweep pose budget exhausted")

// sweepEventOf publishes a contact producer's finished report as the ideal
// event at at.
func sweepEventOf(at vocab.SweepInstant, contact *contactReport) sweepEvent {
	return sweepEvent{At: at, Relation: contact.Relation,
		Gap: contact.Gap, Overlap: contact.Overlap,
		Manifold: contact.Manifold, Reason: contact.Reason}
}

// sampleWith records a sample's exact fraction of the duration.
func sampleWith(s sweepSample, f *big.Rat) sweepSample {
	s.SetExactFraction(seal.Key{}, f)
	return s
}

// sampleWithPtr is sampleWith for a sample kept by pointer.
func sampleWithPtr(s sweepSample, f *big.Rat) *sweepSample {
	s.SetExactFraction(seal.Key{}, f)
	return &s
}

// sweptBoundsOf is the box body b sweeps along p, or false when b's bounds or
// the path's travel bound are not finite.
func sweptBoundsOf(b *topo.Body, p affinePairPath) (sweeppath.SweptBounds, bool) {
	return sweeppath.NodeSweptBounds(b, p)
}
