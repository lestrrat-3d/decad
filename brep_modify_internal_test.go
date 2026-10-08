package decad

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin docs/brep-modify-design.md's receiver dispatch (§2), its
// stage-2a gates (§6), SB2 and SB1, and the refusals a receiver that passes
// both meets outside route P: SB3, SB10, and modify-reach SX16 while route E
// is not built.

// requireBrepModifyRefuses runs Fillet, Chamfer and Shell on body and
// requires each to refuse with ErrUnsupported naming want, leaving body live
// and the document's body set unchanged.
func requireBrepModifyRefuses(t *testing.T, body *Body, want string) {
	t.Helper()
	doc := body.doc
	before := doc.Bodies()
	edges := Edges(Convex()).AtLeast(1)
	_, err := body.Fillet(t.Context(), edges, units.Millimeters(1))
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, want)
	require.ErrorContains(t, err, "fillets")
	_, err = body.Chamfer(t.Context(), edges, units.Millimeters(1))
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, want)
	require.ErrorContains(t, err, "chamfers")
	_, err = body.Shell(t.Context(), Faces(Planar()).AtLeast(1), units.Millimeters(1))
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, want)
	require.ErrorContains(t, err, "shells")
	require.NoError(t, doc.requireLive(body))
	require.Equal(t, before, doc.Bodies())
}

// requireSB1Names requires the SB1 refusal on body and that it names the
// section displacement the record handed to the brep route carries.
func requireSB1Names(t *testing.T, body *Body) {
	t.Helper()
	bp, ok, err := brepModifyRecord(t.Context(), body.payload, "fillets")
	require.NoError(t, err)
	require.True(t, ok)
	delta := bp.sectionDelta()
	require.Positive(t, delta, "the fixture's record carries a section displacement")
	requireBrepModifyRefuses(t, body, "brep-modify SB1")
	_, err = body.Fillet(t.Context(), Edges(Convex()).AtLeast(1), units.Millimeters(1))
	require.ErrorContains(t, err, fmt.Sprintf("displacement of %g mm", delta))
}

// TestBrepModifyOutsideRoutePRefuses pins the refusals that follow route P
// (§2, §6): a selection no prism reading admits falls to route E, which is
// not built, so a Fillet or Chamfer refuses with modify-reach SX16; a Shell
// refuses with SB3 where the body reads as a prism whose caps are not the
// removed faces, and with SB10 where it reads as none. S1's convex edges
// include cap edges, which no prism reading takes; the stacked pocket reads
// as no prism. Every refusal leaves the receiver live. Shown to fail with
// modifyBrepReceiver's shell arms deleted (each Shell read SX16) and with
// its route E refusal replaced by a nil return (each Fillet and Chamfer fell
// through to the generic "straight prism" refusal).
func TestBrepModifyOutsideRoutePRefuses(t *testing.T) {
	t.Parallel()
	refuses := func(t *testing.T, body *Body, shell string) {
		t.Helper()
		before := body.doc.Bodies()
		edges := Edges(Convex()).AtLeast(1)
		_, err := body.Fillet(t.Context(), edges, units.Millimeters(1))
		requireRefusesUnchanged(t, body, before, err, "brep-modify route E", "fillets", "SX16")
		_, err = body.Chamfer(t.Context(), edges, units.Millimeters(1))
		requireRefusesUnchanged(t, body, before, err, "brep-modify route E", "chamfers", "SX16")
		_, err = body.Shell(t.Context(), Faces(Planar()).AtLeast(1), units.Millimeters(1))
		requireRefusesUnchanged(t, body, before, err, shell)
	}
	t.Run("brep", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		refuses(t, s1, "brep-modify SB3")
	})
	t.Run("stacked pocket", func(t *testing.T) {
		t.Parallel()
		_, pocket := internalPocket(t)
		sp, ok := pocket.payload.(stackedPrismPayload)
		require.True(t, ok, "the pocket is a stacked prism, got %T", pocket.payload)
		require.Zero(t, sp.sectionDelta)
		refuses(t, pocket, "brep-modify SB10")
	})
}

// TestBrepModifySB1RefusesADisplacedReceiver pins SB1 (Table RB's RB3): a
// brep or stacked receiver whose record carries a section displacement
// refuses every modify op, naming the displacement. The stacked receiver
// reaches SB1 through its face view rather than the generic non-prism
// refusal. Shown to fail with requireExactBrepSection's call deleted from
// modifyBrepReceiver (every receiver then read SX16), and, for the stacked
// receiver, with modifyBrepReceiver returning nil for a stackedPrismPayload
// (it then read the generic "straight prism" refusal).
func TestBrepModifySB1RefusesADisplacedReceiver(t *testing.T) {
	t.Parallel()
	t.Run("keyway", func(t *testing.T) {
		// general-boolean §9's B2: a Ø20 rod along y, cut by a 4 mm key.
		t.Parallel()
		doc := New()
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XZ(), -20)
		require.NoError(t, err)
		rod := internalClassBTool(t, doc, w, plane, 20, func(s *sketch.Sketch) {
			c := s.CreatePoint(0, 0)
			s.Fix(c)
			s.CreateCircle(c, 10)
		})
		key := internalBoxBodyAtZ(t, doc, -2, -1, 2, 41, 0, 12)
		keyway, err := Cut(t.Context(), rod, key)
		require.NoError(t, err)
		_, ok := keyway.payload.(brepPayload)
		require.True(t, ok, "the keyway is a brep, got %T", keyway.payload)
		requireSB1Names(t, keyway)
	})
	t.Run("crossing round boss", func(t *testing.T) {
		// general-boolean §9's A1 crossing boss: a Ø10 boss at x = 18 crosses
		// the 40×40 plate's x = 20 wall.
		t.Parallel()
		plate, boss := internalBossOnPlate(t, 18, 10, 15)
		union, err := Union(t.Context(), plate, boss)
		require.NoError(t, err)
		_, ok := union.payload.(brepPayload)
		require.True(t, ok, "the crossing boss union is a brep, got %T", union.payload)
		requireSB1Names(t, union)
	})
	t.Run("stacked placed boss", func(t *testing.T) {
		// A boss placed in the plate's plane re-expresses into the plate's
		// frame, so the stacked union carries a section displacement.
		t.Parallel()
		doc := New()
		plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
		boss := internalCircleBody(t, doc, -0.1, 5, 0, Distance{D: units.Millimeters(25), Dir: Along})
		move, err := r3.Translation(r3.NewVec(0.1, 0, 0))
		require.NoError(t, err)
		boss, err = boss.Placed(t.Context(), move)
		require.NoError(t, err)
		union, err := Union(t.Context(), plate, boss)
		require.NoError(t, err)
		sp, ok := union.payload.(stackedPrismPayload)
		require.True(t, ok, "the union is a stacked prism, got %T", union.payload)
		require.Positive(t, sp.sectionDelta)
		requireSB1Names(t, union)
	})
}

// TestBrepModifySB2RefusesAPrismGroup pins SB2: a stacked receiver with no
// face view — a prism group, two disjoint boxes over one interval — refuses
// every modify op with brepOfStacked's ErrUnsupported. Shown to fail with
// brepOfStacked's prism-group refusal deleted (the group's face view then
// built from its first region's caps, and every op read SX16).
func TestBrepModifySB2RefusesAPrismGroup(t *testing.T) {
	t.Parallel()
	doc := New()
	group, sp := internalBoxGroup(t, doc)
	require.Len(t, sp.slabs[0].regions, 2)
	requireBrepModifyRefuses(t, group, "brep-modify SB2")
	_, err := brepOfStacked(t.Context(), sp)
	require.ErrorIs(t, err, ErrUnsupported)
	_, err = group.Fillet(t.Context(), Edges(Convex()).AtLeast(1), units.Millimeters(1))
	require.ErrorContains(t, err, "a prism group of 2 disjoint regions has no face view")
}
