package pairkernel_test

import (
	"context"
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/curvecells"
	"github.com/lestrrat-3d/decad/internal/clearance/spine"
	"github.com/lestrrat-3d/decad/internal/clearance/tier"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file checks the cells that read their exact distance only once the
// trim keeps the candidate (tier.VertexFace's plane arm, tier.VertexEdge's
// line arm, curvecells' line × line edge cell) against copies of the cells
// as they read before: the distance first, then the admission.

// legacyVertexFacePlane is tier.VertexFace's plane arm, reading the distance
// before the sink discards a rejected foot.
func legacyVertexFacePlane(budget *proofbound.WorkBudget, tol float64, v r3.Vec, f *clearance.CFace,
	sink *clearance.CellSink) error {
	h := v.Sub(f.O).Dot(f.N)
	foot := v.Sub(f.N.Scale(h))
	x, y := f.PlaneCoords(foot)
	admit, err := clearance.RegionClassifyBudget(budget, f.Region, x, y, tol)
	if err != nil {
		return err
	}
	d := clearance.Height(v, f.O, f.N).Abs().Widen(clearance.DirCharge([]r3.Vec{f.N}, []r3.Vec{v, f.O}))
	sink.CandidateDist(tol, admit, d, foot, v)
	return nil
}

// legacyVertexEdgeLine is tier.VertexEdge's line arm, reading the distance
// before the admission.
func legacyVertexEdgeLine(tol float64, v r3.Vec, e *clearance.CEdge, sink *clearance.CellSink) {
	dir := e.B.Sub(e.A)
	u, ok := dir.Normalize()
	if !ok {
		return
	}
	foot := clearance.LinePoint(e.A, u, v)
	seg, okSeg := clearance.SegDir(e.A, e.B)
	if !okSeg {
		return
	}
	sink.CandidateDist(tol, clearance.LineParamAdmit(e, foot, tol), clearance.PointLineDist(v, e.A, seg), foot, v)
}

// legacyLineLineEE is curvecells' line × line edge cell, reading the common
// perpendicular's distance before the admission.
func legacyLineLineEE(ctx context.Context, tol, slack float64, ea, eb *clearance.CEdge, sink *clearance.CellSink) {
	oracle := clearance.Oracle{Tol: tol}
	da := ea.B.Sub(ea.A)
	db := eb.B.Sub(eb.A)
	ua, oka := da.Normalize()
	ub, okb := db.Normalize()
	if !oka || !okb {
		return
	}
	segA, okA := clearance.SegDir(ea.A, ea.B)
	segB, okB := clearance.SegDir(eb.A, eb.B)
	if !okA || !okB {
		return
	}
	switch oracle.ParallelSegs(ea.A, ea.B, eb.A, eb.B) {
	case clearance.DegYes:
		p0 := eb.A.Sub(ea.A).Dot(ua)
		p1 := eb.B.Sub(ea.A).Dot(ua)
		w := clearance.NewLinWindow(p0, p1)
		lo := math.Max(0, w.Lo)
		hi := math.Min(da.Len(), w.Hi)
		if hi-lo <= tol {
			return
		}
		t := (lo + hi) / 2
		pa := ea.A.Add(ua.Scale(t))
		pb := clearance.LinePoint(eb.A, ub, pa)
		sink.CandidateDist(tol, 1, clearance.PointLineDist(eb.A, ea.A, segA), pa, pb)
		return
	case clearance.DegUnknown:
		if clearance.ClrBoxDist(ea.Box, eb.Box) > tol {
			return
		}
		sink.Unsure = true
		return
	}
	e := &spine.Engine{Context: ctx, Tolerance: tol, Slack: slack}
	c, ok := e.LineLinePerp(ea.A, ua, eb.A, ub, clearance.LineLineDist(ea.A, segA, eb.A, segB))
	if !ok {
		sink.Coarse(ea.Box, eb.Box, ea.Witnesses(tol), eb.Witnesses(tol))
		return
	}
	admit := clearance.AdmitState(clearance.LineParamAdmit(ea, c.Fa, tol), clearance.LineParamAdmit(eb, c.Fb, tol))
	sink.Candidate(tol, admit, c.Lo, c.Hi, c.Exact, c.Fa, c.Fb)
}

// boxFeatures is one box body's carrier faces, line edges and vertices, read
// the way the pair kernel reads them.
type boxFeatures struct {
	faces []*clearance.CFace
	edges []*clearance.CEdge
	verts []r3.Vec
}

func featuresOf(t *testing.T, body *topo.Body) boxFeatures {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	built, ok, err := clearance.BuildPrismBodyCarriers(budget, body.Payload.(prismrecord.Payload))
	require.NoError(t, err)
	require.True(t, ok)
	out := boxFeatures{faces: built.Faces}
	for _, e := range topobuild.Edges(body) {
		ce, ok := clearance.NewCEdge(e)
		require.True(t, ok)
		if ce != nil {
			out.edges = append(out.edges, ce)
		}
	}
	for _, v := range topobuild.Vertices(body) {
		out.verts = append(out.verts, v.Position)
	}
	return out
}

// deferredScenes is a fixed set of box pairs: seeded random placements, some
// overlapping and some apart, and axis-aligned pairs whose corners and edges
// project exactly onto the other box's face and edge boundaries, so the trim
// both keeps, rejects and straddles candidates.
func deferredScenes(t *testing.T) [][2]boxFeatures {
	t.Helper()
	rng := rand.New(rand.NewPCG(2617, 7))
	randomPlaced := func() r3.Transform {
		axis := r3.NewVec(rng.Float64()*2-1, rng.Float64()*2-1, rng.Float64()*2-1)
		if axis.Len() < 0.1 {
			axis = r3.NewVec(1, 1, 1)
		}
		rot, err := r3.Rotation(axis, units.Radians(rng.Float64()*math.Pi))
		require.NoError(t, err)
		move, err := r3.Translation(r3.NewVec(rng.Float64()*30-15, rng.Float64()*30-15, rng.Float64()*30-15))
		require.NoError(t, err)
		placed, err := rot.Then(move)
		require.NoError(t, err)
		return placed
	}
	var out [][2]boxFeatures
	for range 24 {
		a := placedBox(t, 0, 0, 1+rng.Float64()*20, 1+rng.Float64()*8, 1+rng.Float64()*8, randomPlaced())
		b := placedBox(t, -rng.Float64()*10, -rng.Float64()*10, 1+rng.Float64()*10, 1+rng.Float64()*10, 1+rng.Float64()*10, randomPlaced())
		out = append(out, [2]boxFeatures{featuresOf(t, a), featuresOf(t, b)})
	}
	shift := func(x, y, z float64) r3.Transform {
		m, err := r3.Translation(r3.NewVec(x, y, z))
		require.NoError(t, err)
		return m
	}
	for _, aligned := range [][2]*topo.Body{
		{placedBox(t, 0, 0, 10, 10, 10, r3.Identity()), placedBox(t, 10, -5, 20, 5, 10, shift(0, 0, 20))},
		{placedBox(t, 0, 0, 10, 10, 10, r3.Identity()), placedBox(t, 0, 0, 10, 10, 10, shift(10, 10, 15))},
		{placedBox(t, 0, 0, 10, 10, 10, r3.Identity()), placedBox(t, 2, 2, 8, 8, 4, shift(0, 0, 12))},
	} {
		out = append(out, [2]boxFeatures{featuresOf(t, aligned[0]), featuresOf(t, aligned[1])})
	}
	return out
}

// requireSameSink compares two sinks field by field, floats by their bits.
func requireSameSink(t *testing.T, want, got *clearance.CellSink, what string) {
	t.Helper()
	require.Equal(t, want.Unsure, got.Unsure, what)
	require.Equal(t, want.Overlap, got.Overlap, what)
	require.Len(t, got.Contribs, len(want.Contribs), what)
	for i, w := range want.Contribs {
		g := got.Contribs[i]
		require.Equal(t, math.Float64bits(w.Lo), math.Float64bits(g.Lo), "%s: contribution %d lo", what, i)
		require.Equal(t, math.Float64bits(w.Hi), math.Float64bits(g.Hi), "%s: contribution %d hi", what, i)
		require.Equal(t, w.Exact, g.Exact, "%s: contribution %d exactness", what, i)
	}
}

// cellOutcomes counts what the legacy cells did with their candidates.
type cellOutcomes struct {
	discarded, straddled, kept int
}

func (o *cellOutcomes) count(sink *clearance.CellSink) {
	switch {
	case len(sink.Contribs) == 0 && !sink.Unsure:
		o.discarded++
	case len(sink.Contribs) > 0 && math.IsInf(sink.Contribs[0].Hi, 1):
		o.straddled++
	default:
		o.kept++
	}
}

func countingBudget(steps *int) *proofbound.WorkBudget {
	return &proofbound.WorkBudget{
		StepFn: func() error { *steps++; return nil },
		ErrFn:  func() error { return nil },
	}
}

func TestDeferredReadingsMatchLegacyCells(t *testing.T) {
	t.Parallel()
	const tol, slack = 1e-8, 1e-8
	var vf, ve, ee cellOutcomes
	for si, scene := range deferredScenes(t) {
		for side := range 2 {
			mine, other := scene[side], scene[1-side]
			for vi, v := range mine.verts {
				for fi, f := range other.faces {
					require.Equal(t, clearance.CkPlane, f.Kind)
					what := fmt.Sprintf("scene %d side %d vertex %d face %d", si, side, vi, fi)
					var legacySteps, newSteps int
					want := &clearance.CellSink{}
					require.NoError(t, legacyVertexFacePlane(countingBudget(&legacySteps), tol, v, f, want))
					got := &clearance.CellSink{}
					cell := tier.New(t.Context(), tol, slack)
					require.NoError(t, cell.VertexFace(countingBudget(&newSteps), v, f, got))
					requireSameSink(t, want, got, what)
					require.Equal(t, legacySteps, newSteps, what)
					require.NoError(t, cell.Err(), what)
					require.False(t, cell.Refused(), what)
					vf.count(want)
				}
				for ei, e := range other.edges {
					what := fmt.Sprintf("scene %d side %d vertex %d edge %d", si, side, vi, ei)
					want := &clearance.CellSink{}
					legacyVertexEdgeLine(tol, v, e, want)
					got := &clearance.CellSink{}
					cell := tier.New(t.Context(), tol, slack)
					cell.VertexEdge(v, e, got)
					requireSameSink(t, want, got, what)
					require.NoError(t, cell.Err(), what)
					require.False(t, cell.Refused(), what)
					ve.count(want)
				}
			}
		}
		for ai, ea := range scene[0].edges {
			for bi, eb := range scene[1].edges {
				want := &clearance.CellSink{}
				legacyLineLineEE(t.Context(), tol, slack, ea, eb, want)
				got := &clearance.CellSink{}
				cell := curvecells.New(t.Context(), tol, slack)
				cell.EdgeEdge(ea, eb, got)
				what := fmt.Sprintf("scene %d edges %d %d", si, ai, bi)
				requireSameSink(t, want, got, what)
				require.NoError(t, cell.Err(), what)
				require.False(t, cell.Refused(), what)
				ee.count(want)
			}
		}
	}
	// Every kind of candidate occurs, so the deferred reading is shown to
	// skip only what the sink discards and to keep everything else.
	for name, o := range map[string]cellOutcomes{"vertex face": vf, "vertex edge": ve, "edge pair": ee} {
		require.Positive(t, o.discarded, name)
		require.Positive(t, o.straddled, name)
		require.Positive(t, o.kept, name)
	}
}
