package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestFreeEdgeAndOpenShellReadTheirOwnState is the point of the sheet-body
// vocabulary PR. Every body the evaluator builds today puts exactly two
// faces on every edge, so a missing arm in EdgePredicate.matches's switch
// would return the right answer by accident through the default case and
// every external test would still pass — this test builds the states
// directly, bypassing the evaluator entirely, so the arm has nowhere to
// hide (docs/surface-design.md §2.2, §3.1).
func TestFreeEdgeAndOpenShellReadTheirOwnState(t *testing.T) {
	t.Parallel()

	t.Run("EdgeIsFreeReadsTheAdjacentFaceCount", func(t *testing.T) {
		t.Parallel()
		one := &Face{}
		two := []*Face{{}, {}}
		three := []*Face{{}, {}, {}}

		require.True(t, (&Edge{faces: []*Face{one}}).IsFree(), `one adjacent face is a free edge`)
		require.False(t, (&Edge{faces: two}).IsFree(), `two adjacent faces is interior`)
		require.False(t, (&Edge{faces: three}).IsFree(), `three adjacent faces is non-manifold`)
		require.False(t, (&Edge{}).IsFree(), `zero adjacent faces is not free`)
	})

	t.Run("FreePredicateMatchesOnTheEdgesOwnState", func(t *testing.T) {
		t.Parallel()
		pred := EdgePredicate{kind: predKindFree}
		free := &Edge{faces: []*Face{{}}}
		interior := &Edge{faces: []*Face{{}, {}}}

		require.True(t, pred.matches(free), `predKindFree must match a one-face edge`)
		require.False(t, pred.matches(interior), `predKindFree must not match a two-face edge`)
	})

	t.Run("ShellIsOpenReadsItsOwnStoredField", func(t *testing.T) {
		t.Parallel()
		require.True(t, (&Shell{open: true}).IsOpen())
		require.False(t, (&Shell{}).IsOpen())
	})

	t.Run("BodyKindReadsItsOwnStoredField", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, BodySheet, (&Body{kind: BodySheet}).Kind())
		require.Equal(t, BodySolid, (&Body{}).Kind(), `BodySolid is the iota zero, so the zero value reads solid`)
	})
}

// TestNormalAtRefusesNURBSSurface pins docs/spline-design.md §7: recovering
// the (u, v) of a given point on a NURBSSurface is a root-find, not a closed
// form, so NormalAt has no bound to publish and refuses. A public Extrude of a
// free-form section now reaches a NURBSSurface-tagged Face
// (extrude_freeform_test.go's TestExtrudeFreeformNormalAtRefuses covers that
// path); this test builds the face directly, the way the package's own
// internal tests build a bare *Face elsewhere, to isolate the refusal from any
// build machinery.
func TestNormalAtRefusesNURBSSurface(t *testing.T) {
	t.Parallel()
	f := &Face{surface: NURBSSurface{}}
	_, err := f.NormalAt(r3.NewVec(0, 0, 0))
	require.ErrorIs(t, err, ErrUnsupported,
		`a NURBSSurface has no closed-form normal to publish a bound for`)
}
