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
// bound again and amplifies it. With this option each such facet is split near
// the rim, and the later boolean reads the face's own bound there. Every bound
// the result carries is still proven. The result
// holds more facets, and a face that was split is refused by a Fillet or
// Chamfer that reads it, so a document that edge-finishes its mesh results
// should not set it. A planar face is never split.
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
