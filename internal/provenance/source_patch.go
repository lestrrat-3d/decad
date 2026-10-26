package provenance

// SourcePatch records where one group of a faceted payload came from, one
// step back (agent-docs/operation-history-design.md, Phase 7 Task 7.3). It is
// immutable once built and holds no topology node, body, document, context or
// cache, so sharing the pointer between a group, the face assembled from it
// and every placed copy is safe.
//
// The record only traces origin. It never admits a retained plane and never
// changes a bound, a tag or a public reading; a reader that wants coverage
// reads the face's own tag and bound. Two records are equal only by pointer:
// the Origins of two groups may match while their records differ, so a map
// keyed by Origins alone would merge distinct patches.
type SourcePatch struct {
	// Producer is the operand body's Origin.Producer for a Boolean group, and
	// the modify operation's own producer for a blend group.
	Producer Producer
	// Operand is the operand position in the Boolean (0 for the target, 1
	// for the tool). It is 0 for a blend group.
	Operand int
	// Ordinal is the face's index in the operand's topobuild.Faces order, or
	// the edge's index in the call when Blend is true.
	Ordinal int
	// Blend marks the group a blend minted for one edge of its call.
	Blend bool
	// Parent is the operand face's own record when the operand is faceted,
	// and nil for a built face and for a blend group.
	Parent *SourcePatch
}
