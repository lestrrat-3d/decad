package decad

import (
	"context"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestPrismIntersectArrangedRecordMatchesAuthenticated(t *testing.T) {
	t.Parallel()
	for _, fixture := range []struct {
		name  string
		build func(*testing.T, *Document) (*Body, *Body)
	}{
		{name: "circular washer", build: func(t *testing.T, doc *Document) (*Body, *Body) {
			return internalDiscBody(t, doc, 15, 10), internalWasherBodySymmetric(t, doc, 8, 3, 11)
		}},
		{name: "non-dyadic nested rectangles", build: func(t *testing.T, doc *Document) (*Body, *Body) {
			return prismRectBody(t, doc, 0.1, 0.2, 10.3, 10.4),
				prismRectBody(t, doc, 0.3, 0.4, 3.7, 4.9)
		}},
	} {
		t.Run(fixture.name, func(t *testing.T) {
			doc := New()
			a, b := fixture.build(t, doc)
			pa, pb := a.n.Payload.(prismPayload), b.n.Payload.(prismPayload)
			reexpression, err := prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
			require.NoError(t, err)
			budget := proofbound.NewWorkBudget(t.Context())
			scene, match, _, _, resolved, err := prismcells.ResolveIntersect(t.Context(), budget,
				pa.Profile, pb.Profile, reexpression)
			require.NoError(t, err)
			require.True(t, resolved)
			require.True(t, match.Valid)

			outerBefore := append([]sketch.BoundaryEdge(nil), match.Outer...)
			holesBefore := slices.Clone(match.Holes)
			for i, hole := range match.Holes {
				holesBefore[i] = append([]sketch.BoundaryEdge(nil), hole...)
			}
			authenticated, err := prismcells.RecordProfileContext(t.Context(), scene, match)
			require.NoError(t, err)
			arranged, err := prismcells.RecordArrangedProfileContext(t.Context(), match)
			require.NoError(t, err)
			require.Equal(t, authenticated, arranged)
			require.Equal(t, outerBefore, match.Outer)
			require.Equal(t, holesBefore, match.Holes)

			result, ok, err := resolveAndBuildPrismIntersect(t.Context(),
				proofbound.NewWorkBudget(t.Context()), pa, pb, reexpression)
			require.NoError(t, err)
			require.True(t, ok)
			require.Equal(t, authenticated, result.Profile)

			canceled, cancel := context.WithCancel(t.Context())
			cancel()
			_, err = prismcells.RecordProfileContext(canceled, scene, match)
			require.ErrorIs(t, err, context.Canceled)
			_, err = prismcells.RecordArrangedProfileContext(canceled, match)
			require.ErrorIs(t, err, context.Canceled)
		})
	}
}

func BenchmarkPrismIntersectProfileRecord(b *testing.B) {
	doc := New()
	disc := internalDiscBody(b, doc, 15, 10)
	washer := internalWasherBodySymmetric(b, doc, 8, 3, 11)
	pa, pb := disc.n.Payload.(prismPayload), washer.n.Payload.(prismPayload)
	reexpression, err := prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
	require.NoError(b, err)
	scene, match, _, _, resolved, err := prismcells.ResolveIntersect(b.Context(),
		proofbound.NewWorkBudget(b.Context()), pa.Profile, pb.Profile, reexpression)
	require.NoError(b, err)
	require.True(b, resolved)
	b.Run("authenticated", func(b *testing.B) {
		for b.Loop() {
			_, err := prismcells.RecordProfileContext(b.Context(), scene, match)
			if err != nil {
				b.Fatal(err)
			}
		}
	})
	b.Run("arranged", func(b *testing.B) {
		for b.Loop() {
			_, err := prismcells.RecordArrangedProfileContext(b.Context(), match)
			if err != nil {
				b.Fatal(err)
			}
		}
	})
}
