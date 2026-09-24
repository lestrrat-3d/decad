package decad

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// This file pins docs/surface-design.md's T138, the one chain admission case
// decad's public seam cannot exercise directly: a snapshot match that must
// survive Sketch.Chains() re-ranking a held chain by renaming one of its
// entities, with no geometry change and no re-solve. Every publicly buildable
// chain sits at a unique position under sketch's own coordinate-first ranking
// (chains.go's compareChains), so a rename never actually re-ranks it — the
// one construction that DOES re-rank is sketch's own coincident-duplicate
// fixture (chains_test.go's TestChainsRenameReranksWithoutStaleness), whose
// chains are Chain.Valid == false and so never reach ExtrudeChain at all.
// authenticateChain runs before that Valid gate, so it is exercised directly
// here, over exactly that fixture.

// TestAuthenticateChainMatchesByContentAfterRename is T138: three coincident
// lines from (0,0) to (10,0) rank by name under the tied coordinates
// (chains.go's compareChains rung 2). Renaming the entity the held chain
// walks moves its own chain to a different index in a fresh Sketch.Chains()
// call, changing NOTHING Sketch.Revision() covers, so the held chain stays
// fresh (Chain.IsStale() == false) throughout. authenticateChain must still
// find it — by content, never by the index it once held.
func TestAuthenticateChainMatchesByContentAfterRename(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	var lines []*sketch.Line
	for _, name := range []string{"A", "B", "C"} {
		line := s.CreateLine(s.CreatePoint(0, 0), s.CreatePoint(10, 0))
		line.SetName(name)
		lines = append(lines, line)
	}

	held := s.Chains()
	require.Len(t, held, 3)
	require.Same(t, lines[0], held[0].Entities[0], `"A" sorts first under the tied coordinates`)
	target := held[0]
	require.False(t, target.IsStale())

	// Rename A to Z: no geometry changes, so the revision — and IsStale — is
	// unmoved, but the name-ranked order now sorts B, C, Z: a fresh call's
	// index 0 is a DIFFERENT chain than the one this test is holding.
	lines[0].SetName("Z")
	require.False(t, target.IsStale(), "nothing the held chain HOLDS moved")

	fresh := s.Chains()
	require.NotSame(t, target.Entities[0], fresh[0].Entities[0],
		"a different chain now occupies the held chain's original index")

	// Shown-to-fail: an index-based match — comparing target only against
	// fresh[0] — finds a snapshot mismatch (fresh[0] walks "B", not "A"/"Z"),
	// which is exactly the wrong ErrInvalidProfile T138 names.
	require.False(t, sameChainSnapshot(target, fresh[0]),
		"an index-based read would find the wrong chain at the held index")

	trusted, err := authenticateChain(s, target)
	require.NoError(t, err)
	require.Same(t, lines[0], trusted.Entities[0], "matched by content, not by the stale index")
}
