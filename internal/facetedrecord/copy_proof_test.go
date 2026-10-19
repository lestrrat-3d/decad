package facetedrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// lineageFixture is a live faceted source at a translation and its image one
// further translation along, both in one document.
func lineageFixture(t *testing.T) (*topo.Document, *topo.Body, *topo.Body, r3.Transform, r3.Transform) {
	t.Helper()
	sourcePlacement, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	targetPlacement, err := r3.Translation(r3.NewVec(5, 2, 3))
	require.NoError(t, err)
	doc := &topo.Document{}
	source := &topo.Body{Doc: doc, Origin: provenance.Ref{Producer: 7},
		Payload: facetedrecord.Payload{Xform: sourcePlacement}}
	image := &topo.Body{Doc: doc, Payload: facetedrecord.Payload{Xform: targetPlacement}}
	doc.LiveSet = []*topo.Body{source}
	return doc, source, image, sourcePlacement, targetPlacement
}

// quarterTurn is an exact quarter turn about Z through the origin.
func quarterTurn(t *testing.T) *exactrotation.Rotation {
	t.Helper()
	rec, _, err := exactrotation.New(r3.Vec{}, r3.NewVec(0, 0, 1), units.Radians(math.Pi/2))
	require.NoError(t, err)
	return rec
}

func lineageOf(t *testing.T, b *topo.Body) *facetedrecord.FacetedCopyLineage {
	t.Helper()
	p, ok := b.Payload.(facetedrecord.Payload)
	require.True(t, ok)
	return p.CopyLineage
}

// TestBindCopyLineage pins BindCopyLineage's record and each of its refusals.
//
// Shown to fail first (go test -overlay over copy_proof.go): each subtest goes
// red when its own guard is removed, and the no-inverse subtest when a missing
// inverse is read as the identity. Three mutants of the motion guards survive
// because no input reaches them: ignoring Inverse's error leaves a zero
// transform that Then refuses, and Then validates its own result, so dropping
// either Then's error check or the IsValid check alone changes nothing.
func TestBindCopyLineage(t *testing.T) {
	t.Parallel()

	t.Run("DirectCopyRecordsItsMotion", func(t *testing.T) {
		t.Parallel()
		_, source, image, sourcePlacement, targetPlacement := lineageFixture(t)
		exact := quarterTurn(t)
		facetedrecord.BindCopyLineage(image, source, targetPlacement, true, exact)
		got := lineageOf(t, image)
		require.NotNil(t, got)
		require.Same(t, source, got.Source)
		require.Equal(t, provenance.Producer(7), got.SourceProducer)
		require.Equal(t, sourcePlacement, got.SourcePlacement)
		require.Equal(t, targetPlacement, got.TargetPlacement)
		want, err := r3.Translation(r3.NewVec(4, 0, 0))
		require.NoError(t, err)
		require.Equal(t, want, got.Motion, "the motion takes the source placement to the target")
		require.True(t, got.Direct)
		require.Same(t, exact, got.Exact, "a direct copy keeps its exact rotation")
	})

	t.Run("IndirectCopyIsNotDirect", func(t *testing.T) {
		t.Parallel()
		_, source, image, _, targetPlacement := lineageFixture(t)
		facetedrecord.BindCopyLineage(image, source, targetPlacement, false, quarterTurn(t))
		got := lineageOf(t, image)
		require.NotNil(t, got)
		require.False(t, got.Direct)
		require.Nil(t, got.Exact, "only a direct copy denotes an exact rotation")
	})

	t.Run("RetiredSourceBindsNothing", func(t *testing.T) {
		t.Parallel()
		doc, source, image, _, targetPlacement := lineageFixture(t)
		doc.LiveSet = nil
		facetedrecord.BindCopyLineage(image, source, targetPlacement, true, nil)
		require.Nil(t, lineageOf(t, image))
	})

	t.Run("ForeignImageBindsNothing", func(t *testing.T) {
		t.Parallel()
		_, source, image, _, targetPlacement := lineageFixture(t)
		image.Doc = &topo.Document{}
		facetedrecord.BindCopyLineage(image, source, targetPlacement, true, nil)
		require.Nil(t, lineageOf(t, image))
	})

	t.Run("NilImageBindsNothing", func(t *testing.T) {
		t.Parallel()
		_, source, _, _, targetPlacement := lineageFixture(t)
		facetedrecord.BindCopyLineage(nil, source, targetPlacement, true, nil)
		require.Nil(t, lineageOf(t, source), "the source gains no lineage of its own")
	})

	t.Run("NilSourceBindsNothing", func(t *testing.T) {
		t.Parallel()
		_, _, image, _, targetPlacement := lineageFixture(t)
		facetedrecord.BindCopyLineage(image, nil, targetPlacement, true, nil)
		require.Nil(t, lineageOf(t, image))
	})

	t.Run("SourcePlacementWithoutInverseBindsNothing", func(t *testing.T) {
		t.Parallel()
		_, source, image, _, targetPlacement := lineageFixture(t)
		source.Payload = facetedrecord.Payload{Xform: r3.Transform{}}
		facetedrecord.BindCopyLineage(image, source, targetPlacement, true, nil)
		require.Nil(t, lineageOf(t, image))
	})

	t.Run("PlacementMismatchBindsNothing", func(t *testing.T) {
		t.Parallel()
		_, source, image, sourcePlacement, _ := lineageFixture(t)
		facetedrecord.BindCopyLineage(image, source, sourcePlacement, true, nil)
		require.Nil(t, lineageOf(t, image))
	})
}
