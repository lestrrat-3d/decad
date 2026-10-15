package stitchrecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Unstitch's evaluator (docs/surface-design.md §6.5). It reuses
// internal/surfacegeom's placement transforms and proofbound.FiniteVec
// machinery unchanged — a placed unstitched face is transformed the same way
// a placed stitched one is — and adds the one new mechanism §6.5 calls for: a
// payload that re-evaluates ONE held face of an already-built B-rep under a
// rigid motion, the same idea Payload already carries for a whole
// welded set, restricted here to a single face.

// BuildUnstitchFace is Unstitch's whole evaluator for one face, shared
// by the initial split and every later placement: it places the face's own
// held vertices exactly once, deep-copies fresh topology under the motion so
// nothing aliases the source body, and publishes the result's measurements.
func BuildUnstitchFace(ctx context.Context, env topo.Env, srcFace *topo.Face, srcBounds measurement.Box, xform r3.Transform) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	maxInputAbs := 0.0
	for _, l := range srcFace.Loops {
		for _, ce := range l.CoEdges {
			maxInputAbs = max(maxInputAbs, proofbound.VecMaxAbs(ce.Edge.Start.Position), proofbound.VecMaxAbs(ce.Edge.End.Position))
		}
	}
	delta := 0.0
	if xform != r3.Identity() {
		delta = proofbound.RigidRoundAllow(maxInputAbs, proofbound.VecMaxAbs(xform.Translation()))
	}

	nf, err := copyFaceUnder(ctx, env.Alloc, srcFace, xform, delta)
	if err != nil {
		return nil, err
	}

	body := env.Alloc.NewBody()
	body.Solid = false
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySheet
	body.Doc = env.Doc
	nf.Body = body
	if err := topobuild.AttachFaceLoops(ctx, []*topo.Face{nf}); err != nil {
		return nil, err
	}
	shell := env.Alloc.NewFaceShell()
	shell.Open = topobuild.ShellIsOpen([]*topo.Face{nf})
	shell.Faces = []*topo.Face{nf}
	lump := env.Alloc.NewLump()
	lump.Shells = []*topo.FaceShell{shell}
	body.Lumps = []*topo.Lump{lump}

	// Area is the face's own reading, unaffected by delta: this evaluator
	// never recomputes area from placed coordinates — it copies the analytic
	// value the source face already carried (copyFaceUnder). The
	// placement's held basis is orthonormal only to rounding, so
	// chargePlacement widens the copied area and lengths for it.
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(nf.Area),
		Exactness: measurement.ExactnessOf(nf.AreaBound),
		Bound:     units.SquareMillimeters(nf.AreaBound),
	}
	if err := chargePlacement(body, xform); err != nil {
		return nil, err
	}

	box, err := faceBounds(srcFace, nf, srcBounds, xform, delta)
	if err != nil {
		return nil, err
	}
	body.Bounds = box

	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}

	body.Payload = UnstitchPayload{Xform: xform, Delta: delta, Face: srcFace, Bounds: srcBounds}
	return body, nil
}

// copyFaceUnder deep-copies srcFace's surface, loops, edges and
// vertices under xform, minting fresh vertex, edge and face nodes that
// share no pointer with the retired receiver (docs/surface-design.md §6.5):
// a retired body stays readable, and a caller may still hold it, so mutating
// its topology through a shared pointer would corrupt what they read. It
// mirrors build.go's rebuildTopology per-face walk with no weld plan:
// a single unstitched face never merges an edge or a vertex with anything
// else, so every new edge ends up free by construction once
// topobuild.AttachFaceLoops registers it on this one face alone. A vertex or
// edge reached twice within the SAME face (a coedge revisiting a shared
// corner, or a full circle's own single coedge using its edge once) still
// resolves to the SAME new object, keyed by the old pointer, so the copy
// stays one connected boundary rather than disjoint free-floating pieces.
//
// Every field this copies is read straight off srcFace/its edges/vertices
// verbatim except the surface, curve and vertex position (transformed by
// xform through internal/surfacegeom's placement transforms), the
// vertex bound / edge lengthBound (widened by delta under a non-identity
// placement, exactly as rebuildTopology widens them), the CURVE
// half of the shared-denotation certificate (restated under xform by
// CurveToken.Compose, internal/denotation/token.go, rather than overwritten — this is one
// of the copiers that lets a later Stitch prove an unstitched-and-restitched
// pair coincident again even where a bound makes bit-identity alone fall
// short), topo.Face.AxialDelta, and topo.Face.CapRecord.
//
// topo.Face.AxialDelta, topo.Face.HasAxialDelta and topo.Face.NormalBound carry forward
// rather than reading as absent, because the copy carries the identical
// surface and the identical tag: normalBound states how far that surface
// departs from that tag, and AxialDelta states the same tag's own
// displacement along the same normal, so both readings are exactly as true
// of the copy as they were of srcFace. hasAxialDelta and normalBound copy
// verbatim; AxialDelta additionally widens by delta under a non-identity
// placement, exactly as the vertex bound and edge lengthBound do above.
// normalBound cannot take the same composition: it is DIMENSIONLESS while
// delta is a length, and this package has no term for how far a placement's
// own rounding can rotate the tag frame off the true rotation, so a placed
// copy of a face whose normalBound is nonzero refuses with [decaderr.ErrUnsupported]
// rather than invent one — a narrowing is sound, but inventing an error
// model is not. topo.Face.Denoted, a revolve face's denoted surface, is an exact
// enclosure rather than a dimensionless figure, so it does compose: the copy
// carries its exact image under xform (topo.Face.DenotedUnder). topo.Face.CapRecord,
// the plane a ToFace stop reads, is carried by CapRecord.Under: the same record
// under the identity placement, and none under any other, since a held
// composition of the record with the placement is not the exact image of its
// plane; a copy without a record is not a ToFace stop.
func copyFaceUnder(ctx context.Context, alloc topo.Alloc, srcFace *topo.Face, xform r3.Transform, delta float64) (*topo.Face, error) {
	if xform != r3.Identity() && srcFace.NormalBound != 0 {
		return nil, fmt.Errorf(`%w: a placed copy of a face whose normalBound is nonzero has no dimensionless term to bound the placement's own rotation of the tag frame off the true rotation, so this evaluator refuses rather than guess one`, decaderr.ErrUnsupported)
	}

	newVertByOld := map[*topo.Vertex]*topo.Vertex{}
	vertexFor := func(old *topo.Vertex) (*topo.Vertex, error) {
		if nv, ok := newVertByOld[old]; ok {
			return nv, nil
		}
		p := xform.Apply(old.Position)
		if !proofbound.FiniteVec(p) {
			return nil, fmt.Errorf(`%w: a placed unstitch vertex is not representable`, decaderr.ErrUnsupported)
		}
		bound := old.Bound.Base()
		if delta > 0 {
			bound = proofbound.AbsSumUpper(bound, delta)
		}
		// The CURVE half of the shared-denotation certificate (denotation.go)
		// restates under xform, composing rather than overwriting, so a
		// vertex unstitched and later placed still carries the true
		// accumulated motion.
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

	transformed, err := surfaceplace.TransformSurface(srcFace.Surface, xform)
	if err != nil {
		return nil, err
	}
	axialDelta := srcFace.AxialDelta
	if delta > 0 {
		axialDelta = proofbound.AbsSumUpper(axialDelta, delta)
	}
	nf := topobuild.NewFace(alloc, topo.Face{
		Area:          srcFace.Area,
		AreaBound:     srcFace.AreaBound,
		Reversed:      srcFace.Reversed,
		HeldPlanar:    srcFace.HeldPlanar,
		AxialDelta:    axialDelta,
		HasAxialDelta: srcFace.HasAxialDelta,
		NormalBound:   srcFace.NormalBound,
		Denoted:       srcFace.DenotedUnder(xform),
		Surface:       transformed,
		CapRecord:     srcFace.CapRecord.Under(xform),
		Origins:       append([]provenance.Ref(nil), srcFace.Origins...),
	})
	for _, l := range srcFace.Loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		coedges := make([]topo.CoEdge, len(l.CoEdges))
		for i, ce := range l.CoEdges {
			ne, err := edgeFor(ce.Edge)
			if err != nil {
				return nil, err
			}
			coedges[i] = topo.CoEdge{Edge: ne, Forward: ce.Forward}
		}
		nf.Loops = append(nf.Loops, topobuild.NewLoop(alloc, topo.Loop{Outer: l.Outer, CoEdges: coedges}))
	}
	return nf, nil
}

// faceBounds is Unstitch's own per-face box dispatch. A face bounded
// entirely by straight (Line3) edges on a PLANAR surface has a tight box for
// free: the enclosed region is exactly the polygon its own vertices
// describe, so the axis-aligned extreme along any world axis is always
// attained AT a vertex (a straight edge's interior points are convex
// combinations of its two endpoints, so they never project further than
// either one does) — true of every polygon, convex or not, with or without
// holes. facePolygonBounds computes that box directly off the already-placed
// copy's own held vertex coordinates: no integration, no new proof, just the
// same vertex-derived reading every other planar measurement already
// publishes.
//
// Every other face — a curved surface (Cylinder/Cone/Sphere/Torus/NURBS), or
// a PLANAR surface with any non-Line3 edge (a disk's circular cap is flat but
// its rim bulges past the one vertex a full circle holds) — falls back to
// unstitchBounds, the retiring receiver's own whole-body box. That box is
// sound (a face's own extent is always a subset of the body it came from)
// but not proven tight, and Box has no field that says "sound, not proven
// tight" separately from Exactness/Bound: Exactness here still reads Exact
// whenever xform is the identity and the receiver's own box was, because no
// further numerical rounding was introduced beyond the published numbers —
// not because the box is the tightest one this face could in principle earn.
// docs/surface-design.md §6.5 records that limit.
func faceBounds(srcFace, nf *topo.Face, srcBounds measurement.Box, xform r3.Transform, delta float64) (measurement.Box, error) {
	if (srcFace.Surface.Kind() == surfacegeom.KindPlane || srcFace.HeldPlanar) && faceIsStraightEdged(srcFace) {
		return facePolygonBounds(nf)
	}
	return unstitchBounds(srcBounds, xform, delta)
}

// faceIsStraightEdged reports whether every edge bounding f, across every
// loop, is a Line3 — the one Curve variant a straight-line polygon's own
// bounding box needs (see faceBounds).
func faceIsStraightEdged(f *topo.Face) bool {
	for _, l := range f.Loops {
		for _, ce := range l.CoEdges {
			if _, ok := ce.Edge.Curve.(surfacegeom.Line3); !ok {
				return false
			}
		}
	}
	return true
}

// facePolygonBounds computes a straight-edged planar face's own tight box
// directly off its already-placed vertex set: the componentwise extreme over
// every held coordinate, with Bound the loosest bound any contributing
// vertex itself carries (each vertex's own bound is an isotropic ball, so it
// covers that vertex's contribution to every axis alike). Every vertex
// nf holds was already placed and delta-widened by copyFaceUnder, so
// this needs no separate transform or delta step of its own: an unplaced,
// all-exact face therefore reads Exact with a zero bound, and a placed one
// carries exactly the placement's own rounding — never more.
func facePolygonBounds(nf *topo.Face) (measurement.Box, error) {
	have := false
	var lo, hi r3.Vec
	maxBound := 0.0
	fold := func(v *topo.Vertex) error {
		if !proofbound.FiniteVec(v.Position) {
			return fmt.Errorf(`%w: a placed unstitch vertex is not representable`, decaderr.ErrUnsupported)
		}
		if !have {
			lo, hi = v.Position, v.Position
			have = true
		} else {
			lo = r3.Vec{X: math.Min(lo.X, v.Position.X), Y: math.Min(lo.Y, v.Position.Y), Z: math.Min(lo.Z, v.Position.Z)}
			hi = r3.Vec{X: math.Max(hi.X, v.Position.X), Y: math.Max(hi.Y, v.Position.Y), Z: math.Max(hi.Z, v.Position.Z)}
		}
		maxBound = max(maxBound, v.Bound.Base())
		return nil
	}
	for _, l := range nf.Loops {
		for _, ce := range l.CoEdges {
			if err := fold(ce.Edge.Start); err != nil {
				return measurement.Box{}, err
			}
			if err := fold(ce.Edge.End); err != nil {
				return measurement.Box{}, err
			}
		}
	}
	if !have {
		return measurement.Box{}, fmt.Errorf(`%w: a straight-edged face has no vertex to bound`, decaderr.ErrDegenerate)
	}
	return measurement.Box{Min: lo, Max: hi, Exactness: measurement.ExactnessOf(maxBound), Bound: units.Millimeters(maxBound)}, nil
}

// unstitchBounds publishes one unstitched face's box as the retiring
// receiver's own already-proven Bounds, inflated by its own Bound and then
// carried through xform by its 8 corners (stitchweld.BoxCorners) —
// the same reasoning bounds already states for a curved Stitch
// operand, applied here to a single face of the body being split rather
// than to a whole operand body. The published Bound is exactly delta, on
// the same terms bounds publishes for a placed stitched body. It is
// faceBounds's fallback for every face that is not straight-edged and
// planar (see faceBounds's own doc comment for why the box it returns is
// sound but not necessarily tight).
func unstitchBounds(srcBounds measurement.Box, xform r3.Transform, delta float64) (measurement.Box, error) {
	return stitchweld.UnstitchBounds(srcBounds, xform, delta)
}

// chargePlacement widens a face-copy body's readings for its placement's own
// departure from orthonormal: the copy denotes the source face carried
// through the placement's held basis read exactly, and its area and lengths
// are the source's.
func chargePlacement(body *topo.Body, xform r3.Transform) error {
	l, err := massmoment.PlacementRotation(xform)
	if err != nil {
		return err
	}
	c, err := massmoment.MapChargeOf(l)
	if err != nil {
		return err
	}
	prismrecord.ChargeBodyMap(body, c)
	return nil
}
