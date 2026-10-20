package chainrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// This file builds a chain Extrude's ribbon body from its Payload: one wall
// face per recorded segment of every walk, both rims and the sweep edges, and
// the measurements the finished body publishes (agent-docs/surface-design.md §13).
// It builds a chain of any segment count and kind Table G's per-kind
// construction admits through prismrecord.BuildWallGeometry, shared with the
// profile-fed prism build — the same evaluator over an open rather than a
// closed walk.

// Build builds the ribbon body: one wall face per recorded
// segment of every walk in pp.Chains, both rims of each walk and the one
// sweep edge at each of its two free ends (agent-docs/surface-design.md §13.4,
// Table G row 1), and the measurements the finished body publishes. It
// mirrors prismrecord.Build's own order (internal/prismrecord/build.go) without ever
// building a solid or a closing cap: an open chain mints neither. Each walk
// is its own lump — surface §2.2's one-lump-per-connected-boundary rule,
// which agent-docs/surface-intersection-design.md §3.4 is the first construction
// to exercise with more than one. work is the record's ONE free-form work
// counter (agent-docs/spline-design.md §5.2): the caller opens it once and every
// walk's build and the final bounds reading all spend from it.
func Build(ctx context.Context, env topo.Env, pp Payload, work *freeform.FreeformWork) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(pp.Chains) == 0 {
		return nil, fmt.Errorf(`%w: a ribbon payload holds no walk`, decaderr.ErrDegenerate)
	}
	height := proofbound.BoundedSub(pp.Z1Scalar(), pp.Z0Scalar())
	if height.Value <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, decaderr.ErrDegenerate)
	}

	ref := env.Ref
	body := env.Alloc.NewBody()
	body.Solid = false
	body.Origin = provenance.Ref{Producer: ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySheet
	body.Doc = env.Doc
	var allFaces []*topo.Face
	total := proofbound.BoundedScalar{}
	var captures []WalkCapture
	if pp.SectionDelta == 0 {
		captures = make([]WalkCapture, len(pp.Chains))
	}
	for ci, chain := range pp.Chains {
		var capture *WalkCapture
		if captures != nil {
			capture = &captures[ci]
		}
		faces, area, err := buildSides(ctx, env, body, pp, ci, chain, work, capture)
		if err != nil {
			return nil, err
		}
		allFaces = append(allFaces, faces...)
		total = proofbound.BoundedAdd(total, area)
	}
	body.Lumps = topobuild.SheetLumps(env.Alloc, allFaces)

	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(total.Value),
		Exactness: measurement.ExactnessOf(total.Bound),
		Bound:     units.SquareMillimeters(total.Bound),
	}
	// volume and centroid stay at their zero value: finite, so
	// topobuild.ValidateAnalyticMeasurements below passes, and neither is
	// reachable through Body.Volume/Body.Centroid while solid is false
	// (agent-docs/surface-design.md §8), exactly as patchchain.Build
	// leaves them.
	//
	// A ribbon this design's Trim assembled (pp.SectionDelta != 0 — the only
	// construction that ever sets it, agent-docs/surface-intersection-design.md
	// §3.4) reads its extent through walks that charge §7's δ_cut into
	// exactly the endpoint a cut produced, never into one the record states
	// verbatim (prismcells.TrimProfileWalks). A plain chain Extrude
	// ribbon has zero sectionDelta and reuses the walks resolved for its sides.
	view := pp.Prism()
	var boundsWalks *momentinput.ProfileWalks
	if pp.SectionDelta != 0 {
		var err error
		boundsWalks, err = prismcells.TrimProfileWalks(view.Profile, work)
		if err != nil {
			return nil, err
		}
		if err := boundsWalks.Charge(work); err != nil {
			return nil, err
		}
	} else {
		boundsWalks = BoundsWalks(view.Profile, captures)
	}
	bounds, err := prismrecord.Bounds(ctx, view, work, boundsWalks)
	if err != nil {
		return nil, err
	}
	body.Bounds = bounds
	if err := prismrecord.ChargeMap(body, pp.Frame, pp.Xform); err != nil {
		return nil, err
	}
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	body.Payload = pp
	return body, nil
}

// WalkCapture holds the pre-widening walks one chain's build resolved and the
// free-form work each read spent.
type WalkCapture struct {
	Walks   []survey2d.SegmentWalk
	Charges []momentinput.WalkReadCharge
}

// BoundsWalks reads the exact pre-widening walks buildSides already
// resolved. The cache is local to this build; its per-segment measured charges
// are replayed by momentinput.ResolveOrRead at each bounds read.
func BoundsWalks(profile momentinput.Profile, captures []WalkCapture) *momentinput.ProfileWalks {
	reads := make([][]momentinput.WalkReadCharge, len(captures))
	walks := &momentinput.ProfileWalks{Profile: profile, ReadCharges: reads}
	for i, capture := range captures {
		reads[i] = capture.Charges
		if i == 0 {
			walks.Outer = capture.Walks
		} else {
			walks.Holes = append(walks.Holes, capture.Walks)
		}
	}
	return walks
}

// buildSides builds ONE walk's whole wall set with shared vertices and
// edges: one wall face per recorded segment (Table G row 1), never a cap and
// never a wraparound junction. chainIdx names which of pp.Chains this is,
// feeding prismrecord.SideOrigins's roleLoop exactly as a prism's loop index does,
// so two different walks' faces never collide on one role string. n segments
// place n+1 rim posts at each sweep level, so the walk's two FREE ends (post
// 0 and post n) each carry a sweep edge touched by exactly one wall — Free()
// resolves them (Edge.IsFree, topology.go) — while every INTERIOR post
// (1..n-1) shares its sweep edge between the two walls it joins, exactly as
// prismrecord.BuildLoopSidesAs's own junction verticals do for a closed loop
// (agent-docs/surface-design.md §13.4). It returns the faces and the walk's own
// total wall area, folded through proofbound.BoundedAdd rather than summed as raw
// floats.
func buildSides(ctx context.Context, env topo.Env, body *topo.Body, pp Payload, chainIdx int, chain sectionrecord.ChainRecord, work *freeform.FreeformWork, capture *WalkCapture) ([]*topo.Face, proofbound.BoundedScalar, error) {
	ref := env.Ref
	prismView := pp.Prism()
	// Every coordinate this walk's segments read sits within pp's own section
	// displacement of the section it denotes, so each segment's own length
	// carries that displacement too — prismrecord.BuildLoopSidesAs's identical charge,
	// prism-boolean §7's 12·π·δ per walk (internal/proofbound/bounds.go's
	// proofbound.SectionDisplacementLength), zero for every payload chain Extrude builds
	// directly.
	walkLenAllow := proofbound.SectionDisplacementLength(pp.SectionDelta, 1)
	raw := make([]survey2d.SideWalk, len(chain.Segments))
	if capture != nil {
		capture.Walks = make([]survey2d.SegmentWalk, len(chain.Segments))
		capture.Charges = make([]momentinput.WalkReadCharge, len(chain.Segments))
	}
	for i, seg := range chain.Segments {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		before, beforeRecon := boundarywalk.WorkSpent(work)
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		if capture != nil {
			after, afterRecon := boundarywalk.WorkSpent(work)
			capture.Walks[i] = w
			capture.Charges[i] = momentinput.WalkReadCharge{
				Spent: after - before, ReconstructionSpent: afterRecon - beforeRecon,
			}
		}
		w.LengthBound = proofbound.AbsSumUpper(w.LengthBound, walkLenAllow)
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	walks, err := boundarywalk.CoalesceChainWalksContext(ctx, raw)
	if err != nil {
		return nil, proofbound.BoundedScalar{}, err
	}
	n := len(walks)

	height := proofbound.BoundedSub(pp.Z1Scalar(), pp.Z0Scalar())
	// rimVertex places one rim post at (u, v) on level z, whose own axial
	// displacement is zDelta, beside the post's own exact frame lift and
	// placement rounding (prismPayload.LiftedVertex) — zero wherever that lift
	// is exact, which is what keeps a plain chain Extrude's rim vertices Exact,
	// mirroring prismrecord.BuildLoopSidesAs's own rimVertex.
	rimVertex := func(u, v, z, zDelta, extra float64) *topo.Vertex {
		held, lift := prismView.LiftedVertex(u, v, z)
		return topobuild.NewVertex(env.Alloc, topo.Vertex{Position: held, Bound: units.Millimeters(proofbound.AbsSumUpper(proofbound.AbsSumUpper(zDelta, lift), extra))})
	}

	// Rim posts 0..n, no wraparound: post i sits at walk i's start for
	// i < n, and at the LAST walk's own end for i == n — the chain's two free
	// ends. A free end charges its one walk end's own bound against the point
	// its segment denotes there (boundarywalk.DenotedStartBound and
	// DenotedEndBound); an interior post charges both neighbours' ends
	// (boundarywalk.JunctionVertexAt), exactly as prismrecord.BuildLoopSidesAs does for a closed
	// loop's junctions.
	bottomV := make([]*topo.Vertex, n+1)
	topV := make([]*topo.Vertex, n+1)
	for i := 0; i <= n; i++ {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		var u, v, extra float64
		switch i {
		case n:
			last := walks[n-1]
			u, v = last.EndU, last.EndV
			extra = proofbound.WalkEndBoundAllow(boundarywalk.DenotedEndBound(chain.Segments[last.Segs[len(last.Segs)-1]], last.SegmentWalk))
		case 0:
			u, v = walks[0].StartU, walks[0].StartV
			extra = proofbound.WalkEndBoundAllow(boundarywalk.DenotedStartBound(chain.Segments[walks[0].Segs[0]], walks[0].SegmentWalk))
		default:
			u, v, extra = boundarywalk.JunctionVertexAt(chain.Segments, walks[i-1], walks[i])
		}
		bottomV[i] = rimVertex(u, v, pp.Z0, pp.Z0Delta, extra)
		topV[i] = rimVertex(u, v, pp.Z1, pp.Z1Delta, extra)
	}

	// Sweep edges at every post, free-end and interior alike. Convexity from
	// the 2D turn, exactly as prismrecord.BuildLoopSidesAs's own junction verticals: only
	// an interior post has both a leaving and an entering tangent to cross,
	// so a free end's carries no turn of its own and keeps its zero
	// (concave) default — Table G assigns it no sense.
	vertical := make([]*topo.Edge, n+1)
	for i := 0; i <= n; i++ {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		convex := false
		if i > 0 && i < n {
			prev, w := walks[i-1], walks[i]
			convex = prev.TanOutU*w.TanInV-prev.TanOutV*w.TanInU > 0
		}
		vertical[i] = topobuild.NewEdge(env.Alloc, topo.Edge{Convex: convex, Length: pp.Z1 - pp.Z0, LengthBound: height.Bound, Curve: surfacegeom.Line3{}, Start: bottomV[i], End: topV[i]})
	}

	faces := make([]*topo.Face, 0, n)
	total := proofbound.BoundedScalar{}
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		// holeLoop is always false: a chain has no hole, and the whole walk
		// takes the loop-0 (outer) convention (agent-docs/surface-design.md §13.4).
		convex, err := boundarywalk.RimConvexity(ctx, w, false, work)
		if err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		bottomEdge, topEdge, surf, faceReversed, err := prismrecord.BuildWallGeometry(env.Alloc, prismView, w, convex, false, bottomV[i], bottomV[i+1], topV[i], topV[i+1])
		if err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		origins, err := prismrecord.SideOrigins(ctx, ref, chainIdx, w.Segs)
		if err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		faceArea := proofbound.BoundedMul(proofbound.MeasuredScalar(w.Length, w.LengthBound), height)
		face := topobuild.NewFace(env.Alloc, topo.Face{
			Area:      faceArea.Value,
			AreaBound: faceArea.Bound,
			Reversed:  faceReversed,
			Surface:   surf,
			Origins:   origins,
			Body:      body,
			Loops: []*topo.Loop{topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
				{Edge: bottomEdge, Forward: true},
				{Edge: vertical[i+1], Forward: true},
				{Edge: topEdge, Forward: false},
				{Edge: vertical[i], Forward: false},
			}})},
		})
		for _, e := range []*topo.Edge{bottomEdge, topEdge, vertical[i], vertical[i+1]} {
			e.Faces = append(e.Faces, face)
		}
		faces = append(faces, face)
		total = proofbound.BoundedAdd(total, faceArea)
	}
	return faces, total, nil
}
