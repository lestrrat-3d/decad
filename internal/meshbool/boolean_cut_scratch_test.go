package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The references below preserve the allocating implementation from 3987ea5a.
func referenceRegionOf(tc *TriCutter, poly []int, chainEdges map[[2]int]ChainAnchor) (CutRegion, error) {
	points, err := referenceCollectP2(tc)
	if err != nil {
		return CutRegion{}, err
	}
	tris2, err := EarClipX(tc.Work, points, poly)
	if err != nil {
		return CutRegion{}, err
	}
	if len(tris2) == 0 {
		return CutRegion{}, fmt.Errorf(`%w: a region triangulated to nothing`, decaderr.ErrBooleanFailed)
	}
	reg := CutRegion{}
	for _, t := range tris2 {
		if err := tc.Work.Step(); err != nil {
			return CutRegion{}, err
		}
		corners := [3]proof.Xpt{tc.Verts[t[0]].P3, tc.Verts[t[1]].P3, tc.Verts[t[2]].P3}
		reg.Tris = append(reg.Tris, corners)
		if reg.HasAnchor {
			continue
		}
		for k := range 3 {
			if err := tc.Work.Step(); err != nil {
				return CutRegion{}, err
			}
			key := [2]int{min(t[k], t[(k+1)%3]), max(t[k], t[(k+1)%3])}
			anchor, ok := chainEdges[key]
			if !ok || anchor.ViaParity {
				continue
			}
			reg.HasAnchor = true
			reg.Partner = anchor.Partner
			reg.Probe = XCentroid(corners[0], corners[1], corners[2])
			break
		}
	}
	if !reg.HasAnchor {
		first := reg.Tris[0]
		reg.Probe = XCentroid(first[0], first[1], first[2])
	}
	return reg, nil
}

// referenceCollectP2 preserves the original allocating copy loop.
func referenceCollectP2(tc *TriCutter) ([]Xp2, error) {
	out := make([]Xp2, len(tc.Verts))
	for i, v := range tc.Verts {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		out[i] = v.P2
	}
	return out, nil
}

func regionScratchFixture(exact bool) *TriCutter {
	tc := &TriCutter{}
	for _, v := range []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(4, 4, 0), r3.NewVec(0, 4, 0), r3.NewVec(2, 2, 0)} {
		p := proof.XptOf(v)
		q := NewXP2FromXpt(p, 0, 1)
		if exact {
			q.FloatFinite = false
		}
		tc.Verts = append(tc.Verts, CutVert{P2: q, P3: p})
	}
	return tc
}

func regionScratchSequence(tc *TriCutter, stop int, reuse bool, anchored bool) ([]CutRegion, int, error) {
	calls := 0
	tc.Work = &proofbound.WorkBudget{StepFn: func() error {
		calls++
		if calls == stop {
			return context.Canceled
		}
		return nil
	}, ErrFn: func() error { return nil }}
	anchors := map[[2]int]ChainAnchor{}
	if anchored {
		anchors[[2]int{0, 1}] = ChainAnchor{Partner: [3]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(0, 0, 4)}}
	}
	var scratch []Xp2
	var out []CutRegion
	for _, poly := range [][]int{{0, 1, 2, 3}, {0, 1, 4}, {1, 2, 4}, {2, 3, 4}, {3, 0, 4}} {
		var reg CutRegion
		var err error
		if reuse {
			reg, err = tc.regionOf(poly, anchors, &scratch)
		} else {
			reg, err = referenceRegionOf(tc, poly, anchors)
		}
		if err != nil {
			return nil, calls, err
		}
		out = append(out, reg)
	}
	return out, calls, nil
}

func TestRegionProjectionScratchMatchesAllocating(t *testing.T) {
	for _, exact := range []bool{false, true} {
		for _, anchored := range []bool{false, true} {
			t.Run(fmt.Sprintf("exact=%t/anchored=%t", exact, anchored), func(t *testing.T) {
				want, steps, err := regionScratchSequence(regionScratchFixture(exact), 0, false, anchored)
				require.NoError(t, err)
				got, gotSteps, err := regionScratchSequence(regionScratchFixture(exact), 0, true, anchored)
				require.NoError(t, err)
				require.Equal(t, steps, gotSteps)
				require.Empty(t, regionScratchDifference(want, got))
				// Comparing the entire sequence also checks that later refills did not alter earlier results.
				for stop := 1; stop <= steps+1; stop++ {
					want, n, wantErr := regionScratchSequence(regionScratchFixture(exact), stop, false, anchored)
					got, m, gotErr := regionScratchSequence(regionScratchFixture(exact), stop, true, anchored)
					require.Equal(t, n, m, "stop %d", stop)
					require.Empty(t, regionScratchDifference(want, got), "stop %d", stop)
					if wantErr == nil {
						require.NoError(t, gotErr)
					} else {
						require.ErrorIs(t, gotErr, context.Canceled)
					}
				}
			})
		}
	}
}

func TestProjectionScratchRefillsAndGrows(t *testing.T) {
	tc := regionScratchFixture(false)
	tc.Work = proofbound.NewWorkBudget(t.Context())
	scratch, err := tc.collectP2Into(nil)
	require.NoError(t, err)
	first := &scratch[0]
	scratch[0] = Xp2{}
	again, err := tc.collectP2Into(scratch)
	require.NoError(t, err)
	require.True(t, first == &again[0])
	require.Empty(t, projectionScratchDifference(tc.Verts[0].P2, again[0]))
	tc.Verts = append(tc.Verts, tc.Verts[0])
	grown, err := tc.collectP2Into(again)
	require.NoError(t, err)
	require.Len(t, grown, len(tc.Verts))
	require.Empty(t, projectionScratchDifference(tc.Verts[len(tc.Verts)-1].P2, grown[len(grown)-1]))
	// The public allocating helper still returns independent slice storage.
	a, err := tc.CollectP2()
	require.NoError(t, err)
	b, err := tc.CollectP2()
	require.NoError(t, err)
	a[0] = Xp2{}
	require.Empty(t, projectionScratchDifference(tc.Verts[0].P2, b[0]))
	tc.Verts = nil
	empty, err := tc.CollectP2()
	require.NoError(t, err)
	require.NotNil(t, empty)
	require.Empty(t, empty)
}

func TestRegionProjectionScratchPreservesRefusals(t *testing.T) {
	for _, poly := range [][]int{{0, 3, 2, 1}, {0, 0, 0}} {
		a, b := regionScratchFixture(false), regionScratchFixture(false)
		n, m := 0, 0
		a.Work = &proofbound.WorkBudget{StepFn: func() error { n++; return nil }}
		b.Work = &proofbound.WorkBudget{StepFn: func() error { m++; return nil }}
		want, wantErr := referenceRegionOf(a, poly, nil)
		scratch := make([]Xp2, len(b.Verts))
		got, gotErr := b.regionOf(poly, nil, &scratch)
		require.Error(t, wantErr)
		require.EqualError(t, gotErr, wantErr.Error())
		require.Equal(t, n, m)
		require.Empty(t, regionScratchDifference([]CutRegion{want}, []CutRegion{got}))
	}
}

func scratchIntEqual(a, b *big.Int) bool {
	if a == nil || b == nil {
		return a == b
	}
	return a.Cmp(b) == 0
}
func scratchRatEqual(a, b *big.Rat) bool {
	if a == nil || b == nil {
		return a == b
	}
	return a.Cmp(b) == 0
}
func scratchPointEqual(a, b proof.Xpt) bool {
	return scratchIntEqual(a.X, b.X) && scratchIntEqual(a.Y, b.Y) && scratchIntEqual(a.Z, b.Z) && scratchIntEqual(a.W, b.W)
}
func projectionScratchDifference(a, b Xp2) string {
	for _, field := range []struct {
		name  string
		equal bool
	}{
		{"U", scratchRatEqual(a.U, b.U)}, {"V", scratchRatEqual(a.V, b.V)},
		{"Fu", math.Float64bits(a.Fu) == math.Float64bits(b.Fu)}, {"Fv", math.Float64bits(a.Fv) == math.Float64bits(b.Fv)},
		{"FloatFinite", a.FloatFinite == b.FloatFinite}, {"floatFromExact", a.floatFromExact == b.floatFromExact},
		{"Hu", scratchIntEqual(a.Hu, b.Hu)}, {"Hv", scratchIntEqual(a.Hv, b.Hv)}, {"Hw", scratchIntEqual(a.Hw, b.Hw)},
	} {
		if !field.equal {
			return "projection field " + field.name + " differs"
		}
	}
	return ""
}
func regionScratchDifference(a, b []CutRegion) string {
	if (a == nil) != (b == nil) || len(a) != len(b) {
		return "region nilness or count differs"
	}
	for i := range a {
		x, y := a[i], b[i]
		if x.HasAnchor != y.HasAnchor {
			return fmt.Sprintf("region %d anchor flag differs", i)
		}
		if !scratchPointEqual(x.Probe, y.Probe) {
			return fmt.Sprintf("region %d probe differs", i)
		}
		for j := range x.Partner {
			p, q := x.Partner[j], y.Partner[j]
			if math.Float64bits(p.X) != math.Float64bits(q.X) || math.Float64bits(p.Y) != math.Float64bits(q.Y) || math.Float64bits(p.Z) != math.Float64bits(q.Z) {
				return fmt.Sprintf("region %d partner %d differs", i, j)
			}
		}
		if (x.Tris == nil) != (y.Tris == nil) || len(x.Tris) != len(y.Tris) {
			return fmt.Sprintf("region %d triangle nilness or count differs", i)
		}
		for j := range x.Tris {
			for k := range 3 {
				if !scratchPointEqual(x.Tris[j][k], y.Tris[j][k]) {
					return fmt.Sprintf("region %d triangle %d corner %d differs", i, j, k)
				}
			}
		}
	}
	return ""
}
