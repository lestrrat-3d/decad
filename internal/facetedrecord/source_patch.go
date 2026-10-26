package facetedrecord

import (
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// SourcePatch is the immutable record a Group carries to say which operand
// face or blend it came from (provenance.SourcePatch). It admits nothing.
type SourcePatch = provenance.SourcePatch

// FaceSource builds the record of f, the ordinal-th face of the operand at
// position operand whose body was produced by producer. The parent is f's own
// record when the operand is faceted. f must not be nil.
func FaceSource(producer provenance.Producer, operand, ordinal int, f *topo.Face) *SourcePatch {
	return &SourcePatch{Producer: producer, Operand: operand, Ordinal: ordinal, Parent: f.Source}
}

// BlendSource builds the record of the group a modify operation minted for
// the ordinal-th edge of its call.
func BlendSource(producer provenance.Producer, ordinal int) *SourcePatch {
	return &SourcePatch{Producer: producer, Ordinal: ordinal, Blend: true}
}
