package patchchain

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// RowFacesDisagree is Table R row R18: a chain's adjacent faces cannot be
// brought into agreement on its patch's orientation.
const RowFacesDisagree = `docs/surface-design.md Table R row R18`

// orientedEdge is one chain edge under the orientation decision: the
// receiver's own (read-only) edge, and the sense — forward from Edge.Start
// to Edge.End, or backward — the new patch face traverses it in.
type orientedEdge struct {
	old     *topo.Edge
	forward bool
}

// orientChain reads each edge's adjacent face and delegates the cycle
// walk to Orient. See docs/surface-design.md §5.2.
func orientChain(chain Chain) ([]orientedEdge, error) {
	oriented := make([]OrientedEdge[*topo.Edge, *topo.Vertex], len(chain.Edges))
	for i, e := range chain.Edges {
		if len(e.Faces) != 1 {
			return nil, fmt.Errorf(`%w: a PatchEdges chain edge is not free (%s)`, decaderr.ErrDegenerate, RowFacesDisagree)
		}
		dir, ok := e.Faces[0].CoEdgeDirection(e)
		if !ok {
			return nil, fmt.Errorf(`%w: a PatchEdges chain edge's adjacent face does not use it (%s)`, decaderr.ErrDegenerate, RowFacesDisagree)
		}
		oriented[i] = OrientedEdge[*topo.Edge, *topo.Vertex]{Edge: e, Start: e.Start, End: e.End, Forward: !dir}
	}
	walk, err := Orient(oriented, RowFacesDisagree)
	if err != nil {
		return nil, err
	}
	order := make([]orientedEdge, len(walk))
	for i, oe := range walk {
		order[i] = orientedEdge{old: oe.Edge, forward: oe.Forward}
	}
	return order, nil
}

// BuildBody is PatchEdges's whole evaluator, shared by the initial build and
// every later placement: it rebuilds the receiver's own face set under xform
// (copyFacesUnder), builds one new face per chain on top of that same rebuild
// (buildFace, which re-derives orientation and re-runs gate 4 fresh every
// time), assembles the result, and publishes its measurements.
func BuildBody(ctx context.Context, env topo.Env, srcFaces []*topo.Face, srcBounds measurement.Box, chains []Chain, xform r3.Transform) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	maxInputAbs := 0.0
	for _, f := range srcFaces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				maxInputAbs = max(maxInputAbs, proofbound.VecMaxAbs(ce.Edge.Start.Position), proofbound.VecMaxAbs(ce.Edge.End.Position))
			}
		}
	}
	delta := 0.0
	if xform != r3.Identity() {
		delta = proofbound.RigidRoundAllow(maxInputAbs, proofbound.VecMaxAbs(xform.Translation()))
	}

	newFaces, edgeCopy, err := copyFacesUnder(ctx, env.Alloc, srcFaces, xform, delta)
	if err != nil {
		return nil, err
	}

	patchFaces := make([]*topo.Face, len(chains))
	for i, chain := range chains {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		pf, err := buildFace(ctx, env.Alloc, env.Ref, chain, edgeCopy, xform, delta)
		if err != nil {
			return nil, err
		}
		patchFaces[i] = pf
	}

	allFaces := make([]*topo.Face, 0, len(newFaces)+len(patchFaces))
	allFaces = append(allFaces, newFaces...)
	allFaces = append(allFaces, patchFaces...)
	if err := topobuild.AttachFaceLoops(ctx, allFaces); err != nil {
		return nil, err
	}

	// Kind and lumps/shells come from topobuild.SheetLumps, run only AFTER
	// every face is attached (its own contract). PatchEdges never publishes a
	// solid: closing a sheet's last free edge leaves a CLOSED sheet reporting
	// BodySheet with IsSolid() false, since Stitch alone makes the material
	// claim (docs/surface-design.md §2.1).
	body := env.Alloc.NewBody()
	body.Solid = false
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySheet
	body.Doc = env.Doc
	for _, f := range allFaces {
		f.Body = body
	}
	body.Lumps = topobuild.SheetLumps(env.Alloc, allFaces)

	// Area is the receiver's own faces' area plus each new face's, composed
	// through proofbound.BoundedAdd — never a re-derived reading — exactly the sum
	// the stitch evaluator runs over its own constituent faces
	// (docs/surface-design.md §6.4/§8), reused here for one operand's own
	// face set plus the faces this call adds to it.
	areaAcc := proofbound.BoundedScalar{}
	for _, f := range allFaces {
		areaAcc = proofbound.BoundedAdd(areaAcc, proofbound.MeasuredScalar(f.Area, f.AreaBound))
	}
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(areaAcc.Value),
		Exactness: measurement.ExactnessOf(areaAcc.Bound),
		Bound:     units.SquareMillimeters(areaAcc.Bound),
	}

	// Bounds is the receiver's own already-proven box: every new face's
	// boundary is already receiver geometry, and a bounded planar region
	// lies inside its own boundary's box, so stitchweld.UnstitchBounds's own
	// reasoning — reuse the operand's own extent rather than re-deriving a
	// tighter one — carries unchanged from one face to a whole rebuilt face set.
	bounds, err := stitchweld.UnstitchBounds(srcBounds, xform, delta)
	if err != nil {
		return nil, err
	}
	body.Bounds = bounds

	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}

	body.Payload = BodyPayload{Xform: xform, Delta: delta, Faces: srcFaces, Bounds: srcBounds, Chains: chains}
	return body, nil
}

// copyFacesUnder deep-copies every one of srcFaces under xform, minting
// fresh Vertex, Edge and Face values that share no pointer with the retiring
// receiver (docs/surface-design.md's "rebuild the receiver from its
// payload" decision): a retired body stays readable, and a caller may still
// hold it, so mutating its topology through a shared pointer would corrupt
// what they read.
//
// It is the unstitch face copy widened from one face to a whole
// face SET: an edge or vertex used by two of the receiver's own faces — an
// ordinary interior edge, exactly as common on a sheet as on a solid — still
// resolves to the SAME new object via the returned edgeCopy cache, the same
// per-old-edge sharing the stitch rebuild relies on for its
// own operand faces. Unlike that function, no weld plan is consulted: a
// patch never merges two DIFFERENT edges into one, it only gives an
// already-free edge a second face once buildFace's own coedge is
// attached, so the plain per-pointer cache is the whole of what sharing
// needs here.
//
// Each copy carries its source face's own AxialDelta, HasAxialDelta and
// NormalBound, on the unstitch face copy's terms: the copy holds
// the identical surface and tag, so both figures are as true of it as of
// the source. AxialDelta widens by delta under a non-identity placement, and
// a placed copy of a face whose NormalBound is nonzero refuses with
// [decaderr.ErrUnsupported], since no dimensionless term bounds the placement's own
// rotation of the tag frame. A revolve face's denoted surface (topo.Face.Denoted)
// carries over as its exact image under xform.
func copyFacesUnder(ctx context.Context, alloc topo.Alloc, srcFaces []*topo.Face, xform r3.Transform, delta float64) ([]*topo.Face, map[*topo.Edge]*topo.Edge, error) {
	if xform != r3.Identity() {
		for _, f := range srcFaces {
			if f.NormalBound != 0 {
				return nil, nil, fmt.Errorf(`%w: a placed copy of a face whose normalBound is nonzero has no dimensionless term to bound the placement's own rotation of the tag frame off the true rotation, so this evaluator refuses rather than guess one`, decaderr.ErrUnsupported)
			}
		}
	}
	// The copies denote the source faces carried through the placement's held
	// basis read exactly, which is orthonormal only to rounding, so every
	// copied area and length widens by its defect (massmoment.MapCharge).
	basis, err := massmoment.PlacementRotation(xform)
	if err != nil {
		return nil, nil, err
	}
	place, err := massmoment.MapChargeOf(basis)
	if err != nil {
		return nil, nil, err
	}
	newVertByOld := map[*topo.Vertex]*topo.Vertex{}
	vertexFor := func(old *topo.Vertex) (*topo.Vertex, error) {
		if nv, ok := newVertByOld[old]; ok {
			return nv, nil
		}
		p := xform.Apply(old.Position)
		if !proofbound.FiniteVec(p) {
			return nil, fmt.Errorf(`%w: a placed patch vertex is not representable`, decaderr.ErrUnsupported)
		}
		bound := old.Bound.Base()
		if delta > 0 {
			bound = proofbound.AbsSumUpper(bound, delta)
		}
		// The CURVE half of the shared-denotation certificate (internal/denotation)
		// restates under xform, composing rather than overwriting, exactly
		// as the unstitch face copy does.
		nv := topobuild.NewVertex(alloc, topo.Vertex{Position: p, Bound: units.Millimeters(bound), Denot: old.Denot.Compose(xform)})
		newVertByOld[old] = nv
		return nv, nil
	}

	newEdgeByOld := map[*topo.Edge]*topo.Edge{}
	edgeFor := func(old *topo.Edge) (*topo.Edge, error) {
		if ne, ok := newEdgeByOld[old]; ok {
			return ne, nil
		}
		curve, err := surfaceplace.TransformCurve(old.Curve, xform)
		if err != nil {
			return nil, err
		}
		start, err := vertexFor(old.Start)
		if err != nil {
			return nil, err
		}
		end, err := vertexFor(old.End)
		if err != nil {
			return nil, err
		}
		lengthBound := old.LengthBound
		if delta > 0 {
			lengthBound = proofbound.AbsSumUpper(lengthBound, delta)
		}
		lengthBound = place.LengthBound(old.Length, lengthBound)
		ne := topobuild.NewEdge(alloc, topo.Edge{
			Convex:          old.Convex,
			Length:          old.Length,
			LengthBound:     lengthBound,
			LengthUnbounded: old.LengthUnbounded,
			Denot:           old.Denot.Compose(xform),
			Curve:           curve,
			Start:           start,
			End:             end,
		})
		ne.CurveBound, ne.CurveBounded = surfaceplace.PlacedCurveBound(
			old.Curve, old.CurveBound, old.CurveBounded, curve, xform)
		newEdgeByOld[old] = ne
		return ne, nil
	}

	newFaces := make([]*topo.Face, len(srcFaces))
	for i, f := range srcFaces {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		transformed, err := surfaceplace.TransformSurface(f.Surface, xform)
		if err != nil {
			return nil, nil, err
		}
		axialDelta := f.AxialDelta
		if delta > 0 {
			axialDelta = proofbound.AbsSumUpper(axialDelta, delta)
		}
		nf := topobuild.NewFace(alloc, topo.Face{
			Area:          f.Area,
			AreaBound:     place.AreaOf(proofbound.MeasuredScalar(f.Area, f.AreaBound)).Bound,
			Reversed:      f.Reversed,
			HeldPlanar:    f.HeldPlanar,
			AxialDelta:    axialDelta,
			HasAxialDelta: f.HasAxialDelta,
			NormalBound:   f.NormalBound,
			Denoted:       f.DenotedUnder(xform),
			Surface:       transformed,
			CapRecord:     f.CapRecord.Under(xform),
			Origins:       append([]provenance.Ref(nil), f.Origins...),
		})
		for _, l := range f.Loops {
			coedges := make([]topo.CoEdge, len(l.CoEdges))
			for j, ce := range l.CoEdges {
				ne, err := edgeFor(ce.Edge)
				if err != nil {
					return nil, nil, err
				}
				coedges[j] = topo.CoEdge{Edge: ne, Forward: ce.Forward}
			}
			nf.Loops = append(nf.Loops, topobuild.NewLoop(alloc, topo.Loop{Outer: l.Outer, CoEdges: coedges}))
		}
		newFaces[i] = nf
	}
	return newFaces, newEdgeByOld, nil
}

// buildFace builds one chain's new face: it derives the chain's
// orientation (orientChain) and its plane.
//
// A chain the EXACT arm admitted derives the plane frame's normal and origin
// DETERMINISTICALLY from that same oriented, placed geometry
// (OrientedNormal, walkOrigin) — never by trying a
// candidate and correcting its sign from a computed area, which would
// silently absorb a wrong orientation instead of surfacing it. This is
// sound because gate 3's exact arm already proved these SAME held vertex
// coordinates bit-for-bit coplanar (dyadic.go's exact rational lift), so
// fitting a normal to them fits one to data that genuinely, exactly, is
// coplanar.
//
// A chain the LEVEL arm admitted instead reads the plane's normal and
// origin from the chain's own level token, transformed by this evaluation's
// own placement (xform) — never fitted to held vertex coordinates, which
// gate 3's level arm never proved exactly coplanar (internal/denotation's own doc
// comment states why a fit would be unsound here). OrientedNormal
// still runs for such a chain, but only to settle which of the token's two
// normal directions (±) matches the chain's own walk sense
// (LevelNormal) — its MAGNITUDE, and any tilt a fit to
// approximately-coplanar data would carry, are discarded.
//
// Either way this re-runs gate 4's crossing audit, and reads the face's area
// from the same moments engine PatchProfile's own single planar face uses.
func buildFace(ctx context.Context, alloc topo.Alloc, ref provenance.Producer, chain Chain, edgeCopy map[*topo.Edge]*topo.Edge, xform r3.Transform, delta float64) (*topo.Face, error) {
	ordered, err := orientChain(chain)
	if err != nil {
		return nil, err
	}

	fitted, err := OrientedNormal(geometryEdges(ordered, edgeCopy))
	if err != nil {
		return nil, err
	}

	normal := fitted
	origin := walkOrigin(ordered, edgeCopy)
	axialDelta := chain.AxialBound
	if chain.HasAxialBound {
		normal, err = LevelNormal(fitted, xform.ApplyDir(chain.Level.Normal))
		if err != nil {
			return nil, err
		}
		origin = xform.Apply(chain.Level.Origin)
		// A placement's own rounding (proofbound.RigidRoundAllow) displaces the token's
		// origin exactly as it displaces every other placed coordinate this
		// evaluator publishes (copyFacesUnder's own vertex/edge
		// widening), so it folds into the SAME AxialDelta the token's own
		// bound already states.
		if delta > 0 {
			axialDelta = proofbound.AbsSumUpper(axialDelta, delta)
		}
	}
	frame, segs, err := frameAndSegments(ordered, edgeCopy, origin, normal)
	if err != nil {
		return nil, err
	}

	ig, err := chainIntegrals(ctx, segs)
	if err != nil {
		return nil, err
	}
	if ig.Area <= 0 {
		return nil, fmt.Errorf(`%w: a PatchEdges chain encloses no area`, decaderr.ErrDegenerate)
	}
	// The integrals are taken in the fitted frame's plane coordinates, which
	// that frame's held axes carry to world orthonormal only to rounding.
	frameMap, err := massmoment.PrismRotation(frame, r3.Identity())
	if err != nil {
		return nil, err
	}
	frameCharge, err := massmoment.MapChargeOf(frameMap)
	if err != nil {
		return nil, err
	}
	ig.AreaBound = frameCharge.AreaOf(proofbound.MeasuredScalar(ig.Area, ig.AreaBound)).Bound
	// A straight-edged rim is a polygon whose corners are the chain's own
	// vertices, so its area is bounded directly in world coordinates, which
	// also charges the float re-expression of the rim into the fitted frame
	// that the plane-coordinate integrals above cannot see.
	areaEdges := make([]AreaEdge, len(ordered))
	for i, oe := range ordered {
		ne := edgeCopy[oe.old]
		areaEdges[i] = AreaEdge{
			Curve: ne.Curve, Forward: oe.forward,
			Start:  AreaVertex{Position: ne.Start.Position, Bound: ne.Start.Bound.Base()},
			End:    AreaVertex{Position: ne.End.Position, Bound: ne.End.Bound.Base()},
			Length: ne.Length, LengthBound: ne.LengthBound, LengthUnbounded: ne.LengthUnbounded,
			CurveBound: ne.CurveBound, CurveBounded: ne.CurveBounded,
		}
	}
	if bound, ok := PolygonAreaBound(ig.Area, areaEdges); ok {
		ig.AreaBound = bound
	} else {
		// A rim with a circular edge is bounded against the loop the
		// integrals read, lifted through the fitted frame, edge by edge
		// (CurvedAreaCharge); a rim it cannot bound refuses rather than
		// publish an area its own truth may sit outside of.
		charge, ok := CurvedAreaCharge(frame, frameMap, frameCharge.Stretch, segs, areaEdges)
		if !ok {
			return nil, fmt.Errorf(`%w: a PatchEdges rim edge carries no proven bound on the curve it denotes, so its face area has none`, decaderr.ErrUnsupported)
		}
		ig.AreaBound = proofbound.AbsSumUpper(ig.AreaBound, charge)
	}

	budget := proofbound.NewWorkBudget(ctx)
	segEntries, err := sectionaudit.EntriesOf(budget, []sectionrecord.LoopRecord{{Segments: segs}})
	if err != nil {
		return nil, remapCrossingError(err)
	}
	if err := sectionaudit.Crossing(budget, segEntries); err != nil {
		return nil, remapCrossingError(err)
	}

	coedges := make([]topo.CoEdge, len(ordered))
	for i, oe := range ordered {
		coedges[i] = topo.CoEdge{Edge: edgeCopy[oe.old], Forward: oe.forward}
	}

	return topobuild.NewFace(alloc, topo.Face{
		Area:      ig.Area,
		AreaBound: ig.AreaBound,
		// A chain gate 3's level arm admitted has a proven-bounded plane
		// origin, never a zero one: the same fields internal/prismrecord/build.go already
		// sets on a prism's own caps (docs/surface-design.md §5.2). A chain
		// the exact arm admitted instead carries a zero, unset bound here.
		AxialDelta:    axialDelta,
		HasAxialDelta: chain.HasAxialBound,
		Surface:       surfacegeom.Plane{Frame: frame},
		Origins:       []provenance.Ref{{Producer: ref, Role: RolePatch}},
		Loops:         []*topo.Loop{topobuild.NewLoop(alloc, topo.Loop{Outer: true, CoEdges: coedges})},
	}), nil
}

// chainIntegrals runs the same closed-form region integral
// PatchProfile's own single planar face uses (internal/momentregion), over a
// single-loop, no-hole record — reused rather than a bespoke shoelace sum,
// so a patch face's Area answers on the identical Exactness/Bound terms
// docs/surface-design.md §5.1 already states for a sketch-recorded patch.
func chainIntegrals(ctx context.Context, segs []sectionrecord.CurveSegment) (momentinput.Integrals, error) {
	work := freeform.NewFreeformWork()
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}.EvaluatorIntegralsContext(ctx, freeform.MomentAreaOrder, work)
}

// remapCrossingError maps the crossing audit's own [decaderr.ErrUnsupported]
// refusal to [decaderr.ErrDegenerate] at gate 4's own boundary
// (docs/surface-design.md §5.2, Table R row R5): a self-crossing chain is
// bad input this evaluator will never admit under a finer tolerance, unlike
// the evaluator-reach refusals ErrUnsupported names elsewhere, so it takes
// the sentinel a caller should branch on as unfixable rather than staged.
func remapCrossingError(err error) error {
	if err == nil {
		return nil
	}
	if errors.Is(err, decaderr.ErrUnsupported) {
		return fmt.Errorf(`%w: a PatchEdges chain's plane-local walk crosses or touches itself (docs/surface-design.md Table R row R5)`, decaderr.ErrDegenerate)
	}
	return err
}

// walkOrigin is the exact arm's own plane origin: the chain's
// first (already placed) vertex in walk order. A chain the exact arm
// admitted has this vertex bit-for-bit coplanar with every other chain
// vertex (gate 3's own dyadic proof), so using it as the plane's origin
// introduces no coordinate this evaluator has not already verified.
func walkOrigin(ordered []orientedEdge, edgeCopy map[*topo.Edge]*topo.Edge) r3.Vec {
	first := edgeCopy[ordered[0].old]
	if !ordered[0].forward {
		return first.End.Position
	}
	return first.Start.Position
}

// frameAndSegments adapts placed edges to plane-local records. SegmentsInFrame
// owns the curved edge sense rule.
func frameAndSegments(ordered []orientedEdge, edgeCopy map[*topo.Edge]*topo.Edge, origin, normal r3.Vec) (r3.Frame, []sectionrecord.CurveSegment, error) {
	frame, err := PlaneFrameFromNormal(origin, normal)
	if err != nil {
		return r3.Frame{}, nil, err
	}
	segs, err := SegmentsInFrame(frame, geometryEdges(ordered, edgeCopy))
	if err != nil {
		return r3.Frame{}, nil, err
	}
	return frame, segs, nil
}

// geometryEdges adapts the rebuilt topology to the ordered geometry read.
func geometryEdges(ordered []orientedEdge, edgeCopy map[*topo.Edge]*topo.Edge) []GeometryEdge {
	out := make([]GeometryEdge, len(ordered))
	for i, oe := range ordered {
		ne := edgeCopy[oe.old]
		g := GeometryEdge{Start: ne.Start.Position, End: ne.End.Position, Forward: oe.forward}
		switch c := ne.Curve.(type) {
		case surfacegeom.Line3:
			g.Kind = Line
		case surfacegeom.Circle3:
			g.Kind, g.Center, g.Axis, g.Radius = Circle, c.Center, c.Axis, c.Radius
		case surfacegeom.Arc3:
			g.Kind, g.Center, g.Axis = Arc, c.Center, c.Axis
		default:
			g.Kind, g.CurveName = Unsupported, publicCurveName(c)
		}
		out[i] = g
	}
	return out
}

// publicCurveName is the type name of the root package's public form of c, as
// the diagnostic for a curve kind a patch cannot represent has always printed
// it: the root Curve variants carry the carrier's own names, and a carrier
// without a public variant reads as a borrowed curve.
func publicCurveName(c surfacegeom.Curve) string {
	switch c.(type) {
	case nil:
		return "<nil>"
	case surfacegeom.Ellipse3:
		return "decad.Ellipse3"
	case surfacegeom.FilletMiter3:
		return "decad.FilletMiter3"
	case surfacegeom.NURBSCurve:
		return "decad.NURBSCurve"
	case surfacegeom.FacetedCurve:
		return "decad.FacetedCurve"
	default:
		return "decad.borrowedCurve"
	}
}
