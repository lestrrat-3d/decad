package decad

import (
	"github.com/lestrrat-go/option/v3"
)

// rimSplitOption is the DocumentOption WithRimSplit returns.
type rimSplitOption struct{ option.Interface }

func (rimSplitOption) documentOption() {}

type identRimSplit struct{}

// WithRimSplit makes a document's mesh booleans keep a rim's bound beside the
// rim. A boolean that cuts a curved face leaves a rim whose bound is larger
// than the face's own, and the facets the rim touches run on to the far end of
// the face, so a later boolean that crosses them far from the rim reads that
// bound again and amplifies it. With this option such a facet is split near the
// rim, and the later boolean reads the face's own bound on the far piece. A
// facet is split only where the whole far piece is proven, by an exact
// distance, to stand clear of the other operand's boundary by the bounds both
// operands hold; a facet with no such split stays whole and keeps the rim's
// bound. The result holds more facets. A Fillet or Chamfer that reads a split
// face may refuse it, since a split adds a vertex that no boundary loop of the
// face reaches; one that succeeds reads the split facets' own bounds, which are
// proven in the same way. A document that edge-finishes its mesh results
// should leave the option off. A planar face is never split.
func WithRimSplit() DocumentOption {
	return rimSplitOption{option.New(identRimSplit{}, true)}
}

// rimSplitOf resolves a document's options to whether they ask for rim
// splitting.
func rimSplitOf(opts []DocumentOption) bool {
	var on bool
	for _, opt := range opts {
		if c, ok := opt.(rimSplitOption); ok && c.Interface != nil {
			on = true
		}
	}
	return on
}
