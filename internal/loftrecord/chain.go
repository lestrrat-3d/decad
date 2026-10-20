package loftrecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// ChainPayload is LoftChain's own record of a ribbon body: the two
// recorded open walks, the two planes and frames they lift through, and the
// accumulated rigid placement. It stays DISTINCT from Payload because a
// chain-fed build mints no cap, carries no volume and runs no whole-shell
// orientation step, so a consumer dispatching on Payload would read three
// facts about it that are not true.
type ChainPayload struct {
	Chain0, Chain1 sectionrecord.ChainRecord
	Plane0, Plane1 sectionrecord.PlaneRecord
	Frame0, Frame1 r3.Frame
	Xform          r3.Transform
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/verifypublish), the name the evaluator has always published for it.
func (ChainPayload) SurveyClass() string { return "chainLoftPayload" }

// Transform is the accumulated rigid placement.
func (lp ChainPayload) Transform() r3.Transform { return lp.Xform }

// ProvesSimple implements topo.SimpleProver. A chain loft records only the
// held flat ribbon triangles. Its build runs the complete crossing audit
// before committing a body, and placement rebuilds and audits that triangle
// set again. A committed chain loft thus already proves leg 4 over the exact
// sheet Verify reads.
func (lp ChainPayload) ProvesSimple(context.Context) bool { return true }

// Placed re-evaluates the same two records under the composed motion (core
// §8), re-running every gate decided from the records and rebuilding from
// scratch, exactly as docs/loft-design.md §4 states a loft placement does.
func (lp ChainPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	lp.Xform = composed
	return BuildChain(ctx, env, lp, proofbound.NewWorkBudget(ctx), freeform.NewFreeformWork(), freeform.NewFreeformWork())
}

// BuildChain builds the ribbon body: the two station chains, the two
// wall triangles per chord cell, the crossing audit over that set, the B-rep
// topology with no cap, and the two readings a sheet publishes
// (docs/loft-design.md §16.5).
//
// It never runs §5's whole-shell orientation step. That step reads the signed
// tetrahedron sum over a CLOSED triangle set, which this build does not
// assemble; loftmesh.ChainLoftPlaneSideGate is what states the side instead, and the
// local winding below is emitted unflipped because that gate has already
// proven it correct.
func BuildChain(ctx context.Context, env topo.Env, lp ChainPayload, budget *proofbound.WorkBudget, work0, work1 *freeform.FreeformWork) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	walks0, walks1, err := loftmesh.ValidateChainLoftRecords(lp.Chain0, lp.Chain1, lp.Plane0, lp.Plane1, work0, work1)
	if err != nil {
		return nil, err
	}
	v, w, stationRound, err := loftmesh.ChainLoftStations(walks0, walks1)
	if err != nil {
		return nil, err
	}

	// S13, decided before the first coordinate is lifted into an exact dyadic,
	// exactly as loftmesh.AssembleTriangulated decides it: tessellation.OrientationSign is not run here,
	// but the mass accumulator and the audit both lift every vertex through
	// dyVec, whose mustDyOf PANICS on a non-finite float.
	anchor := lp.Xform.Apply(lp.Plane0.Origin)
	if !proofbound.FiniteVec(anchor) {
		return nil, loftmesh.ErrPointUnrepresentable("placed plane origin")
	}

	verts := make([]r3.Vec, 0, 2*len(v))
	maxInputAbs := 0.0
	for _, side := range []struct {
		pts   []sectionrecord.Point2
		frame r3.Frame
		what  string
	}{{v, lp.Frame0, "first"}, {w, lp.Frame1, "second"}} {
		for j, pt := range side.pts {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			lifted := side.frame.ToWorldUV(pt.U, pt.V)
			maxInputAbs = math.Max(maxInputAbs, proofbound.VecMaxAbs(lifted))
			placed := lp.Xform.Apply(lifted)
			if !proofbound.FiniteVec(placed) {
				return nil, loftmesh.ErrPointUnrepresentable(fmt.Sprintf("placed station %d on the %s walk", j, side.what))
			}
			verts = append(verts, placed)
		}
	}

	// placeAllow is zero exactly when xform is the identity transform — an
	// exact struct comparison, never a tolerance — and stationRound is the
	// independent leg beside it, exactly as loftmesh.AssembleTriangulated composes the two.
	placeAllow := 0.0
	if lp.Xform != r3.Identity() {
		placeAllow = proofbound.RigidRoundAllow(maxInputAbs, proofbound.VecMaxAbs(lp.Xform.Translation()))
	}
	delta := proofbound.AbsSumUpper(stationRound, placeAllow)

	n := len(v) - 1
	vIdx := make([]int, len(v))
	wIdx := make([]int, len(w))
	for j := range v {
		vIdx[j] = j
		wIdx[j] = len(v) + j
	}
	tris := make([][3]int, 0, 2*n)
	for j := range n {
		tris = append(tris,
			[3]int{vIdx[j], vIdx[j+1], wIdx[j+1]},
			[3]int{vIdx[j], wIdx[j+1], wIdx[j]},
		)
	}

	if err := loftmesh.LoftCrossingAudit(budget, verts, tris); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	body := env.Alloc.NewBody()
	body.Solid = false
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySheet
	body.Doc = env.Doc
	walls, err := buildChainTopology(ctx, env, body, verts, tris, vIdx, wIdx, delta)
	if err != nil {
		return nil, err
	}
	if err := topobuild.AttachFaceLoops(ctx, walls); err != nil {
		return nil, err
	}
	body.Lumps = topobuild.SheetLumps(env.Alloc, walls)

	mass := loftmesh.NewMassAccumulator(anchor, delta, 0, 0)
	for _, t := range tris {
		mass.Add(verts[t[0]], verts[t[1]], verts[t[2]], true)
	}
	// No cap rational is passed: this build mints no cap, so the reading is
	// the wall sum alone rather than a sum with the two caps subtracted back
	// off it the way a surface-result Loft's is (docs/loft-design.md §16.5).
	body.Area = massArea(mass)
	bounds, ok := massBounds(mass)
	if !ok {
		return nil, fmt.Errorf(`%w: the chain loft has no vertices to bound`, decaderr.ErrDegenerate)
	}
	body.Bounds = bounds
	// volume and centroid stay at their zero value: finite, so the validation
	// below passes, and neither is reachable through Body.Volume/Body.Centroid
	// while solid is false, exactly as chainrecord.Build leaves them.
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	body.Payload = lp
	return body, nil
}

// buildChainTopology builds the ribbon's B-rep over the assembled triangle
// set: the same four edge families BuildTopology builds for a closed loft
// — bottom rim, top rim, diagonal and rung — with the wrap dropped everywhere.
//
// An n-cell walk carries n bottom rims, n top rims, n diagonals and n+1 rungs.
// Every diagonal and every INTERIOR rung bounds two triangles; the n rims of
// each walk and the two END rungs bound one each, which is the 2n+2 free edges
// docs/loft-design.md §16.5 states. No loop is closed back to its first
// station and no cap loop is built, so nothing here reads a wrap index.
//
// A free end rung takes the concave default rather than a measured sense: only
// an interior rung has two incident triangles to cross, so a free end carries
// no turn of its own — the identical rule chainrecord's buildSides applies to a chain
// prism's own free-end sweep edges. Each walk's rims take the outer-loop
// convexity convention, since a chain walk is loop 0 throughout
// (docs/surface-design.md §13.4).
func buildChainTopology(ctx context.Context, env topo.Env, body *topo.Body, verts []r3.Vec, tris [][3]int, vIdx, wIdx []int, delta float64) ([]*topo.Face, error) {
	n := len(vIdx) - 1
	vertexObjs := make([]*topo.Vertex, len(verts))
	for i, p := range verts {
		vertexObjs[i] = topobuild.LoftVertex(env.Alloc, p, delta)
	}

	lowerTri := make([][3]int, n)
	upperTri := make([][3]int, n)
	for j := range n {
		lowerTri[j] = tris[2*j]
		upperTri[j] = tris[2*j+1]
	}

	rimBottom := make([]*topo.Edge, n)
	rimTop := make([]*topo.Edge, n)
	diagE := make([]*topo.Edge, n)
	rungE := make([]*topo.Edge, n+1)
	for j := range n {
		rimBottom[j] = topobuild.LoftEdge(env.Alloc, vertexObjs, verts, vIdx[j], vIdx[j+1], true, delta)
		rimTop[j] = topobuild.LoftEdge(env.Alloc, vertexObjs, verts, wIdx[j], wIdx[j+1], true, delta)
		diagE[j] = topobuild.LoftEdge(env.Alloc, vertexObjs, verts, vIdx[j], wIdx[j+1],
			loftmesh.JunctionConvex(verts, lowerTri[j], upperTri[j], vIdx[j], wIdx[j+1]), delta)
	}
	for j := 0; j <= n; j++ {
		convex := false
		if j > 0 && j < n {
			convex = loftmesh.JunctionConvex(verts, lowerTri[j-1], upperTri[j], vIdx[j], wIdx[j])
		}
		rungE[j] = topobuild.LoftEdge(env.Alloc, vertexObjs, verts, vIdx[j], wIdx[j], convex, delta)
	}

	walls := make([]*topo.Face, 0, 2*n)
	for j := range n {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		lowerFace, err := buildWallFace(env, body, verts, lowerTri[j], 0, j, 0, delta, nil)
		if err != nil {
			return nil, err
		}
		lowerFace.Loops = []*topo.Loop{topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
			{Edge: rimBottom[j], Forward: true},
			{Edge: rungE[j+1], Forward: true},
			{Edge: diagE[j], Forward: false},
		},
		})}
		walls = append(walls, lowerFace)

		upperFace, err := buildWallFace(env, body, verts, upperTri[j], 0, j, 1, delta, nil)
		if err != nil {
			return nil, err
		}
		upperFace.Loops = []*topo.Loop{topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
			{Edge: diagE[j], Forward: true},
			{Edge: rimTop[j], Forward: false},
			{Edge: rungE[j], Forward: false},
		},
		})}
		walls = append(walls, upperFace)
	}
	return walls, nil
}
