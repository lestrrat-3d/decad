package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
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
		one := bornFace(&Face{})
		two := []*Face{bornFace(&Face{}), bornFace(&Face{})}
		three := []*Face{bornFace(&Face{}), bornFace(&Face{}), bornFace(&Face{})}

		require.True(t, (&Edge{n: topo.Edge{Faces: []*topo.Face{faceNode(one)}}}).IsFree(), `one adjacent face is a free edge`)
		require.False(t, (&Edge{n: topo.Edge{Faces: faceNodes(two)}}).IsFree(), `two adjacent faces is interior`)
		require.False(t, (&Edge{n: topo.Edge{Faces: faceNodes(three)}}).IsFree(), `three adjacent faces is non-manifold`)
		require.False(t, (&Edge{}).IsFree(), `zero adjacent faces is not free`)
	})

	t.Run("FreePredicateMatchesOnTheEdgesOwnState", func(t *testing.T) {
		t.Parallel()
		pred := EdgePredicate{kind: selectorquery.FreeKind}
		free := &Edge{n: topo.Edge{Faces: []*topo.Face{faceNode(bornFace(&Face{}))}}}
		interior := &Edge{n: topo.Edge{Faces: []*topo.Face{faceNode(bornFace(&Face{})), faceNode(bornFace(&Face{}))}}}

		require.True(t, pred.matches(free), `selectorquery.FreeKind must match a one-face edge`)
		require.False(t, pred.matches(interior), `selectorquery.FreeKind must not match a two-face edge`)
	})

	t.Run("ShellIsOpenReadsItsOwnStoredField", func(t *testing.T) {
		t.Parallel()
		require.True(t, (&FaceShell{n: topo.FaceShell{Open: true}}).IsOpen())
		require.False(t, (&FaceShell{}).IsOpen())
	})

	t.Run("BodyKindReadsItsOwnStoredField", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, BodySheet, (bornBody(&Body{n: topo.Body{Kind: topo.BodySheet}})).Kind())
		require.Equal(t, BodySolid, (bornBody(&Body{})).Kind(), `BodySolid is the iota zero, so the zero value reads solid`)
	})
}

// TestNormalAtRefusesNURBSSurface pins docs/spline-design.md §7: recovering
// the (u, v) of a given point on a NURBSSurface is a root-find, not a closed
// form, so NormalAt has no bound to publish and refuses. A public Extrude of a
// free-form section now reaches a NURBSSurface-tagged Face
// (apitest/extrude_freeform_test.go's TestExtrudeFreeformNormalAtRefuses covers that
// path); this test builds the face directly, the way the package's own
// internal tests build a bare *Face elsewhere, to isolate the refusal from any
// build machinery.
func TestNormalAtRefusesNURBSSurface(t *testing.T) {
	t.Parallel()
	f := bornFace(&Face{n: topo.Face{Surface: surfacegeom.NURBSSurface{}}})
	_, err := f.NormalAt(r3.NewVec(0, 0, 0))
	require.ErrorIs(t, err, ErrUnsupported,
		`a NURBSSurface has no closed-form normal to publish a bound for`)
}

// TestFaceOriginsConvertsTheNodeRoles pins the public form of the provenance
// the face node holds (docs/evaluator-design.md §3): Origins returns nil when
// the node holds no role, as the copy it replaced did, and a held role comes
// back as the matching FeatureRef.
func TestFaceOriginsConvertsTheNodeRoles(t *testing.T) {
	t.Parallel()

	t.Run("NoRolesIsNil", func(t *testing.T) {
		t.Parallel()
		require.Nil(t, (bornFace(&Face{})).Origins())
	})

	t.Run("EmptyNodeSliceIsNil", func(t *testing.T) {
		t.Parallel()
		f := bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{}}})
		require.Nil(t, f.Origins())
	})

	t.Run("HeldRoleIsItsFeatureRef", func(t *testing.T) {
		t.Parallel()
		f := bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Producer: 3, Role: "capEnd"}}}})
		require.Equal(t, []FeatureRef{{producer: 3, Role: "capEnd"}}, f.Origins())
	})

	t.Run("ReturnsACopy", func(t *testing.T) {
		t.Parallel()
		f := bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Producer: 3, Role: "capEnd"}}}})
		f.Origins()[0].Role = "changed"
		require.Equal(t, "capEnd", f.n.Origins[0].Role)
	})

	t.Run("RefConvertersRoundTrip", func(t *testing.T) {
		t.Parallel()
		x := FeatureRef{producer: 7, Role: "capStart"}
		require.Equal(t, x, featureRefOf(refOf(x)))
	})
}

// TestTwinEnumsMatch pins the constant values the root BodyKind and Exactness
// share with their node forms: the generated conversions between the two are
// plain integer casts (docs/evaluator-design.md §3).
func TestTwinEnumsMatch(t *testing.T) {
	t.Parallel()

	require.Equal(t, int(BodySolid), int(topo.BodySolid))
	require.Equal(t, int(BodySheet), int(topo.BodySheet))
	require.Equal(t, int(Exact), int(measurement.Exact))
	require.Equal(t, int(Approximate), int(measurement.Approximate))
}
