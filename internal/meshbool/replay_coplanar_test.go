package meshbool

import (
	"context"
	"errors"
	"fmt"
	"sort"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// keptSignature is a kept facet's identity independent of the order the passes
// emit it in: its corner keys sorted, its source face, and its operand.
func keptSignature(f KeptFacet) [5]string {
	keys := []string{f.V[0].Key(), f.V[1].Key(), f.V[2].Key()}
	sort.Strings(keys)
	second := "A"
	if f.Second {
		second = "B"
	}
	return [5]string{keys[0], keys[1], keys[2], string(rune('0' + f.Src%10)), second}
}

func requireSameKept(t *testing.T, live, replay []KeptFacet) {
	t.Helper()
	require.Len(t, replay, len(live), "kept facet count")
	count := map[[5]string]int{}
	for _, f := range live {
		count[keptSignature(f)]++
	}
	for _, f := range replay {
		count[keptSignature(f)]--
	}
	for k, n := range count {
		require.Zero(t, n, "kept facet %v is not held the same way by both passes", k)
	}
}

type replayCase struct {
	name string
	a, b func(t *testing.T) *BoolMesh
	ops  []OperationKind
}

// replayCoplanarCases are the pairs of §14 Task 12's test: A1, A5 and R3.
func replayCoplanarCases() []replayCase {
	box := func(lo, hi r3.Vec) func(t *testing.T) *BoolMesh {
		return func(t *testing.T) *BoolMesh { return exactBoxMesh(t, lo, hi) }
	}
	all := []OperationKind{OpUnion, OpIntersect, OpCut}
	return []replayCase{
		{"A1", box(r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10}), box(r3.Vec{X: 0.5, Z: 10}, r3.Vec{X: 10.5, Y: 10, Z: 20}), all},
		{"A5", box(r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10}), box(r3.Vec{X: 2, Y: 2, Z: 5}, r3.Vec{X: 8, Y: 8, Z: 10}), all},
		{"R3", box(r3.Vec{}, r3.Vec{X: 20, Y: 16, Z: 10}),
			func(t *testing.T) *BoolMesh {
				// A trapezoid tenon resting on the plate, flush with its y faces.
				return polygonPrismMesh(t, [][2]float64{{5.5, 0}, {14.5, 0}, {16.5, 16}, {3.5, 16}}, 10, 16)
			}, all},
	}
}

// replayPrepared prepares the live operand's facets again on the replay meter,
// with the same bounds and exactness.
func replayPrepared(t *testing.T, ctx context.Context, bm *BoolMesh, budget *proofbound.ReplayBudget) *BoolMesh {
	t.Helper()
	out, err := prepContextReplay(ctx, bm.Verts, bm.Tris, bm.Src, budget)
	require.NoError(t, err)
	out.SetVertexBounds(make([]float64, len(out.Verts)))
	out.Exact = append([]bool(nil), bm.Exact...)
	return out
}

func replayBoolean(t *testing.T, ctx context.Context, op OperationKind, live, other *BoolMesh, budget *proofbound.ReplayBudget) ([]KeptFacet, *ContactKeys, error) {
	t.Helper()
	a, b := replayPrepared(t, ctx, live, budget), replayPrepared(t, ctx, other, budget)
	keys, err := NewReplayKeysBounded(ctx, budget)
	require.NoError(t, err)
	memo, err := NewReplayContactMemo(ctx, a, b, budget)
	require.NoError(t, err)
	kept, _, contact, err := MeshBooleanReplayWithKeys(ctx, op, a, b, memo, budget, keys)
	return kept, contact, err
}

// TestReplayCoplanarMatchesLive pins that the bounded pass classifies a coplanar
// pair of exact facets as the live pass does: the same kept facets for every
// operation, and the same stitched mesh.
func TestReplayCoplanarMatchesLive(t *testing.T) {
	for _, tc := range replayCoplanarCases() {
		for _, op := range tc.ops {
			t.Run(fmt.Sprintf("%s/%v", tc.name, op), func(t *testing.T) {
				a, b := tc.a(t), tc.b(t)
				live, stats, err := MeshBooleanStats(t.Context(), op, a, b, NewContactMemo(a, b))
				require.NoError(t, err)

				ra, rb := tc.a(t), tc.b(t)
				got, contact, err := replayBoolean(t, t.Context(), op, ra, rb, proofbound.NewReplayTrialBudget())
				require.NoError(t, err)
				requireSameKept(t, live, got)
				require.Equal(t, stats.Coplanar != nil, contact != nil)

				if len(live) == 0 {
					return // boundaries that only touch leave nothing to stitch
				}
				want, err := StitchFacetsWith(t.Context(), live, StitchOptions{Coplanar: stats.Coplanar, SimplifyPlanar: true})
				require.NoError(t, err)
				budget := proofbound.NewReplayTrialBudget()
				keys, err := NewReplayKeysBounded(t.Context(), budget)
				require.NoError(t, err)
				stitched, err := StitchFacetsReplayContact(t.Context(), got, budget, keys, contact)
				if errors.Is(err, errReplaySimplifiedMeshNeedsAFlip) {
					t.Skip("the live stitch flips a needle the bounded stitch declines")
				}
				require.NoError(t, err)
				require.Equal(t, len(want.Tris), len(stitched.Tris))
				require.Equal(t, len(want.Verts), len(stitched.Verts))
				require.Equal(t, want.Round, stitched.Round)
			})
		}
	}
}

// TestReplayCoplanarBudget pins that a meter too small for the coplanar pass
// returns the budget error: the least work the bounded boolean needs, less one,
// runs out inside the pass, and is never read as a refusal of the contact.
func TestReplayCoplanarBudget(t *testing.T) {
	a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	b := exactBoxMesh(t, r3.Vec{X: 0.5, Z: 10}, r3.Vec{X: 10.5, Y: 10, Z: 20})
	run := func(limit uint64) error {
		budget, err := proofbound.NewReplayBudgetLimits(limit, proofbound.ReplayTrialRecords, proofbound.ReplayTrialBytes)
		require.NoError(t, err)
		_, _, err = replayBoolean(t, t.Context(), OpUnion, a, b, budget)
		return err
	}
	full := proofbound.NewReplayTrialBudget()
	_, _, err := replayBoolean(t, t.Context(), OpUnion, a, b, full)
	require.NoError(t, err)
	total, _ := full.Usage()
	lo, hi := uint64(1), total
	for lo < hi {
		mid := (lo + hi) / 2
		if run(mid) == nil {
			hi = mid
		} else {
			lo = mid + 1
		}
	}
	require.NoError(t, run(lo))
	for _, limit := range []uint64{lo - 1, lo / 2, lo * 3 / 4} {
		var wl *proofbound.WorkLimitError
		require.ErrorAs(t, run(limit), &wl, "limit %d", limit)
	}
}

// TestReplayCoplanarDeclinesWhatLiveClassifies pins two refusals: a partial
// footprint whose region probes lie on axis planes is built or declined, never a
// hard failure; and a pair whose chains branch refuses as the live pass does,
// with a contact error the shifted retry reads.
func TestReplayCoplanarDeclinesWhatLiveClassifies(t *testing.T) {
	t.Run("partial footprint", func(t *testing.T) {
		a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
		b := exactBoxMesh(t, r3.Vec{X: 5, Y: 5, Z: 10}, r3.Vec{X: 15, Y: 15, Z: 20})
		for _, op := range []OperationKind{OpUnion, OpIntersect, OpCut} {
			_, _, err := replayBoolean(t, t.Context(), op, a, b, proofbound.NewReplayTrialBudget())
			if err != nil {
				require.ErrorIs(t, err, decaderr.ErrUnsupported, "%v is a decline", op)
			}
		}
	})
	t.Run("branching chains", func(t *testing.T) {
		a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
		b := polygonPrismMesh(t, [][2]float64{{14, 8}, {7, 8}, {10, 5}, {10, 2}, {14, 2}}, 2, 8)
		for _, op := range []OperationKind{OpUnion, OpIntersect, OpCut} {
			_, _, liveErr := MeshBooleanStats(t.Context(), op, a, b, NewContactMemo(a, b))
			var want *TouchContactError
			if !errors.As(liveErr, &want) {
				continue // the live pass builds this one; the fixture only pins refusals
			}
			_, _, err := replayBoolean(t, t.Context(), op, a, b, proofbound.NewReplayTrialBudget())
			var have *TouchContactError
			require.ErrorAs(t, err, &have, "%v", op)
			require.Equal(t, want.What, have.What)
		}
	})
}

// TestReplayCoplanarLegacyRefusal pins that boxes meeting along an edge or at a
// corner stitch to the same legacy refusal on the bounded path as on the live
// one, so the retry reads the same contacts.
func TestReplayCoplanarLegacyRefusal(t *testing.T) {
	for name, lo := range map[string]r3.Vec{"edge only": {X: 10, Y: 0.5, Z: 10}, "vertex only": {X: 10, Y: 10, Z: 10}} {
		t.Run(name, func(t *testing.T) {
			a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
			b := exactBoxMesh(t, lo, r3.Vec{X: lo.X + 10, Y: lo.Y + 10, Z: lo.Z + 10})
			live, stats, err := MeshBooleanStats(t.Context(), OpUnion, a, b, NewContactMemo(a, b))
			require.NoError(t, err)
			_, wantErr := StitchFacetsWith(t.Context(), live, StitchOptions{Coplanar: stats.Coplanar})
			var want *TouchContactError
			require.ErrorAs(t, wantErr, &want)

			budget := proofbound.NewReplayTrialBudget()
			got, contact, err := replayBoolean(t, t.Context(), OpUnion, a, b, budget)
			require.NoError(t, err)
			keys, err := NewReplayKeysBounded(t.Context(), budget)
			require.NoError(t, err)
			_, gotErr := StitchFacetsReplayContact(t.Context(), got, budget, keys, contact)
			var have *TouchContactError
			require.ErrorAs(t, gotErr, &have)
			require.Equal(t, want.What, have.What)
			require.Equal(t, want.Contacts, have.Contacts)
		})
	}
}
