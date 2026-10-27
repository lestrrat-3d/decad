package meshbool

import (
	"context"
	"fmt"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// countingBudget is a work budget that never runs out and counts its steps.
func countingBudget(ctx context.Context, steps *int) *proofbound.WorkBudget {
	return &proofbound.WorkBudget{
		StepFn: func() error {
			*steps++
			return nil
		},
		ErrFn: ctx.Err,
	}
}

// legacyCut is TriCutter.cut as it was before chains carried their probes:
// every ChainProbe is computed afresh. Test reference only.
func legacyCut(ctx context.Context, tc *TriCutter, xtri [3]proof.Xpt, normal proof.Xpt, segs []Xseg) ([]CutRegion, error) {
	tc.U, tc.V = ProjAxes(normal)
	cornerIdx, err := tc.internFacet(xtri, segs)
	if err != nil {
		return nil, err
	}
	if len(tc.Edges) == 0 {
		probe := XCentroid(xtri[0], xtri[1], xtri[2])
		return []CutRegion{{Tris: [][3]proof.Xpt{xtri}, Probe: probe}}, nil
	}
	lines, chains, err := tc.OpenLoops()
	if err != nil {
		return nil, err
	}
	pieces := [][]int{{cornerIdx[0], cornerIdx[1], cornerIdx[2]}}
	for _, c := range lines {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		var next [][]int
		for _, piece := range pieces {
			left, right, err := tc.SplitConvexByU(piece, c)
			if err != nil {
				return nil, err
			}
			if left != nil {
				next = append(next, left)
			}
			if right != nil {
				next = append(next, right)
			}
		}
		pieces = next
	}
	byPiece := make([][]ChainPath, len(pieces))
	for _, ch := range chains {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		probe, err := tc.ChainProbe(ch)
		if err != nil {
			return nil, err
		}
		placed := false
		for pi, piece := range pieces {
			pts, err := tc.PolyPoints(piece)
			if err != nil {
				return nil, err
			}
			inside, onBoundary, err := PointInPoly2(tc.Work, pts, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary {
				return nil, fmt.Errorf(`%w: a contact chain lies on a subdivision boundary`, decaderr.ErrBooleanFailed)
			}
			if inside {
				byPiece[pi] = append(byPiece[pi], ch)
				placed = true
				break
			}
		}
		if !placed {
			return nil, fmt.Errorf(`%w: a contact chain escaped every subdivision piece`, decaderr.ErrBooleanFailed)
		}
	}
	chainEdges := tc.anchors
	for _, e := range tc.Edges {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		chainEdges[[2]int{min(e.A, e.B), max(e.A, e.B)}] = ChainAnchor{Partner: e.Partner, ViaParity: e.ViaParity}
	}
	var regions []CutRegion
	for pi, piece := range pieces {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		polys, err := legacySplitByChains(tc, piece, byPiece[pi])
		if err != nil {
			return nil, err
		}
		for _, poly := range polys {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			reg, err := tc.RegionOf(poly, chainEdges)
			if err != nil {
				return nil, err
			}
			regions = append(regions, reg)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return regions, nil
}

// legacySplitByChains is TriCutter.SplitByChains computing every probe with
// ChainProbe. Test reference only.
func legacySplitByChains(tc *TriCutter, piece []int, chains []ChainPath) ([][]int, error) {
	type work struct {
		poly   []int
		chains []ChainPath
	}
	stack := []work{{poly: piece, chains: chains}}
	var out [][]int
	for len(stack) > 0 {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		w := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		if len(w.chains) == 0 {
			out = append(out, w.poly)
			continue
		}
		ch := w.chains[0]
		rest := w.chains[1:]
		poly, err := tc.InsertOnBoundary(w.poly, ch.Verts[0])
		if err != nil {
			return nil, err
		}
		poly, err = tc.InsertOnBoundary(poly, ch.Verts[len(ch.Verts)-1])
		if err != nil {
			return nil, err
		}
		i, err := tc.IndexOf(poly, ch.Verts[0])
		if err != nil {
			return nil, err
		}
		j, err := tc.IndexOf(poly, ch.Verts[len(ch.Verts)-1])
		if err != nil {
			return nil, err
		}
		if i < 0 || j < 0 || i == j {
			return nil, fmt.Errorf(`%w: a chain endpoint is missing from its region boundary`, decaderr.ErrBooleanFailed)
		}
		interior := ch.Verts[1 : len(ch.Verts)-1]
		var polyA []int
		for k := i; ; k = (k + 1) % len(poly) {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyA = append(polyA, poly[k])
			if k == j {
				break
			}
		}
		for _, vi := range slices.Backward(interior) {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyA = append(polyA, vi)
		}
		var polyB []int
		for k := j; ; k = (k + 1) % len(poly) {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyB = append(polyB, poly[k])
			if k == i {
				break
			}
		}
		for _, vi := range interior {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyB = append(polyB, vi)
		}
		ptsA, err := tc.PolyPoints(polyA)
		if err != nil {
			return nil, err
		}
		areaA, err := PolyArea2Sign(tc.Work, ptsA)
		if err != nil {
			return nil, err
		}
		ptsB, err := tc.PolyPoints(polyB)
		if err != nil {
			return nil, err
		}
		areaB, err := PolyArea2Sign(tc.Work, ptsB)
		if err != nil {
			return nil, err
		}
		if areaA <= 0 || areaB <= 0 {
			return nil, fmt.Errorf(`%w: a chain split produced a non-positive region`, decaderr.ErrBooleanFailed)
		}
		wa := work{poly: polyA}
		wb := work{poly: polyB}
		for _, rc := range rest {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			probe, err := tc.ChainProbe(rc)
			if err != nil {
				return nil, err
			}
			inside, onBoundary, err := PointInPoly2(tc.Work, ptsA, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary {
				return nil, fmt.Errorf(`%w: a chain lies on a freshly split boundary`, decaderr.ErrBooleanFailed)
			}
			if inside {
				wa.chains = append(wa.chains, rc)
				continue
			}
			inside, onBoundary, err = PointInPoly2(tc.Work, ptsB, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary || !inside {
				return nil, fmt.Errorf(`%w: a chain escaped both split regions`, decaderr.ErrBooleanFailed)
			}
			wb.chains = append(wb.chains, rc)
		}
		stack = append(stack, wa, wb)
	}
	return out, nil
}

// sameXpt reports whether two exact points hold equal integers, coordinate by
// coordinate: the same representation, not only the same value.
func sameXpt(a, b proof.Xpt) bool {
	return a.X.Cmp(b.X) == 0 && a.Y.Cmp(b.Y) == 0 && a.Z.Cmp(b.Z) == 0 && a.W.Cmp(b.W) == 0
}

// requireSameRegions compares two region lists integer for integer and
// reports only indices on a mismatch.
func requireSameRegions(t *testing.T, want, got []CutRegion, label string) {
	t.Helper()
	require.Len(t, got, len(want), label)
	for r := range want {
		require.Equal(t, want[r].HasAnchor, got[r].HasAnchor, "%s region %d anchor", label, r)
		require.Equal(t, want[r].Partner, got[r].Partner, "%s region %d partner", label, r)
		require.True(t, sameXpt(want[r].Probe, got[r].Probe), "%s region %d probe", label, r)
		require.Len(t, got[r].Tris, len(want[r].Tris), "%s region %d", label, r)
		for k := range want[r].Tris {
			for c := range 3 {
				require.True(t, sameXpt(want[r].Tris[k][c], got[r].Tris[k][c]), "%s region %d tri %d corner %d", label, r, k, c)
			}
		}
	}
}

// punchedFacets is one large facet with every non-empty subset of three boxes
// pushed through its interior. Each box leaves a closed loop, so the cut
// opens it with split lines and adds vertices before chains are placed.
func punchedFacets(t *testing.T) []cutFixtureFacet {
	t.Helper()
	m, err := PrepContext(t.Context(), []r3.Vec{{}, {X: 10}, {Y: 10}}, [][3]int{{0, 1, 2}}, []int{0})
	require.NoError(t, err)
	boxes := []*BoolMesh{
		boxMesh(t, r3.Vec{X: 0.8, Y: 0.6, Z: 1.7}, func(v r3.Vec) r3.Vec { return v }, r3.Vec{X: 2.5, Y: 2.5, Z: 0.125}),
		boxMesh(t, r3.Vec{X: 0.5, Y: 0.4, Z: 1.7}, func(v r3.Vec) r3.Vec { return v }, r3.Vec{X: 5.5, Y: 1.5, Z: 0.25}),
		boxMesh(t, r3.Vec{X: 0.3, Y: 0.7, Z: 1.7}, func(v r3.Vec) r3.Vec { return v }, r3.Vec{X: 1.5, Y: 6, Z: -0.375}),
	}
	var out []cutFixtureFacet
	for subset := 1; subset < 1<<len(boxes); subset++ {
		f := cutFixtureFacet{xtri: XtriCorners(m, 0), normal: m.Norms[0]}
		for bi, b := range boxes {
			if subset&(1<<bi) == 0 {
				continue
			}
			for j := range b.Tris {
				contact, err := TriTriClassify(TriCorners(m, 0), TriCorners(b, j), XtriCorners(m, 0), XtriCorners(b, j), m.Norms[0], b.Norms[j])
				require.NoError(t, err)
				if contact.Kind == ContactSegment {
					f.segs = append(f.segs, Xseg{A: contact.P0, B: contact.P1, Partner: TriCorners(b, j)})
				}
			}
		}
		tc := newTriCutter(t.Context(), nil)
		tc.U, tc.V = ProjAxes(f.normal)
		_, err := tc.internFacet(f.xtri, f.segs)
		require.NoError(t, err)
		lines, _, err := tc.OpenLoops()
		require.NoError(t, err)
		require.NotEmpty(t, lines, "subset %b opens no loop", subset)
		out = append(out, f)
	}
	return out
}

// chainProbeFixtures are the facet sets the recorded-probe tests cut: chains
// of one segment and of several, one chain to a facet and up to a dozen, and
// closed loops opened by split lines.
func chainProbeFixtures(t *testing.T) map[string][]cutFixtureFacet {
	t.Helper()
	out := map[string][]cutFixtureFacet{
		"one-row strips":   cutManyChainFacets(t, 4, 6, 9, 6),
		"wavy grids":       cutFixtureFacets(t, 12),
		"fine corrugation": cutManyChainFacets(t, 3, 4, 0.3, 4),
		"punched":          punchedFacets(t),
	}
	if !raceDetector {
		out["long strips"] = cutManyChainFacets(t, 4, 6, 25, 6)
		out["wavy grids 24"] = cutFixtureFacets(t, 24)
	}
	return out
}

// TestCutRecordedProbesMatchLegacyCut cuts every fixture facet with the
// recorded chain probes and with the reference that recomputes each probe:
// the same regions, integer for integer, the same error and the same number
// of budget steps.
func TestCutRecordedProbesMatchLegacyCut(t *testing.T) {
	t.Parallel()
	multi := 0
	for name, facets := range chainProbeFixtures(t) {
		require.NotEmpty(t, facets, name)
		var scratch cutProjectionScratch
		for fi, f := range facets {
			label := fmt.Sprintf("%s facet %d", name, fi)
			var wantSteps, gotSteps int
			ref := newTriCutter(t.Context(), nil)
			ref.Work = countingBudget(t.Context(), &wantSteps)
			want, wantErr := legacyCut(t.Context(), ref, f.xtri, f.normal, f.segs)
			tc := newTriCutter(t.Context(), &scratch)
			tc.Work = countingBudget(t.Context(), &gotSteps)
			got, gotErr := tc.cut(t.Context(), f.xtri, f.normal, f.segs)
			require.Equal(t, wantErr, gotErr, label)
			require.NoError(t, gotErr, label)
			require.Equal(t, wantSteps, gotSteps, "%s steps", label)
			requireSameRegions(t, want, got, label)
			if len(want) > 2 {
				multi++
			}
		}
	}
	require.Greater(t, multi, 20)
}

// TestSplitByChainsWithoutRecordedProbes splits pieces along chains that
// carry no recorded probe, as an outside caller of SplitByChains builds
// them: the same polygons and steps as the reference.
func TestSplitByChainsWithoutRecordedProbes(t *testing.T) {
	t.Parallel()
	checked := 0
	for _, f := range cutManyChainFacets(t, 4, 6, 9, 6) {
		run := func(split func(tc *TriCutter, piece []int, chains []ChainPath) ([][]int, error)) ([][]int, int) {
			var steps int
			tc := newTriCutter(t.Context(), nil)
			tc.Work = countingBudget(t.Context(), &steps)
			tc.U, tc.V = ProjAxes(f.normal)
			corners, err := tc.internFacet(f.xtri, f.segs)
			require.NoError(t, err)
			lines, chains, err := tc.OpenLoops()
			require.NoError(t, err)
			require.Empty(t, lines)
			polys, err := split(tc, corners[:], chains)
			require.NoError(t, err)
			return polys, steps
		}
		want, wantSteps := run(legacySplitByChains)
		got, gotSteps := run((*TriCutter).SplitByChains)
		require.Equal(t, want, got)
		require.Equal(t, wantSteps, gotSteps)
		if len(want) > 2 {
			checked++
		}
	}
	require.Greater(t, checked, 20)
}
