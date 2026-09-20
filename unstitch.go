package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Unstitch of docs/surface-design.md §6.5: the inverse of
// stitch.go's Stitch, splitting a body back into one single-face sheet body
// per face. It reuses stitch.go's transformSurface/transformCurve/finiteVec
// machinery unchanged — a placed unstitched face is transformed the same way
// a placed stitched one is — and adds the one new mechanism §6.5 calls for: a
// payload that re-evaluates ONE held face of an already-built B-rep under a
// rigid motion, the same idea stitchPayload already carries for a whole
// welded set, restricted here to a single face. Document.commitMany
// (document.go) is the multi-produce atomic commit this file's N results
// need beside the existing single-body commit.

// Unstitch calls [Body.UnstitchContext] with [context.Background].
func (b *Body) Unstitch() ([]*Body, error) {
	return b.UnstitchContext(context.Background())
}

// UnstitchContext returns one single-face sheet body per face of the
// receiver, in [Body.Faces] order, retiring the receiver
// (docs/surface-design.md §6.5). Each result carries that face's own
// surface, loops and readings, and every edge of every result is free: a
// fresh Face, Edge and Vertex set is minted for each result rather than
// reusing the receiver's own topology, so the retired receiver stays
// readable and unmodified for any caller still holding it.
//
// It admits an analytic body, solid or sheet — the asymmetry with [Stitch],
// which takes sheets only (Table R row R17), is deliberate: Unstitch is how
// a caller gets sheets to work with in the first place. A Faceted body — a
// mesh boolean's result — is [ErrUnsupported] (R11): its faces are chord
// polygons with no analytic identity, and returning thousands of facet
// sheets would be a shape nobody asked for. A nil receiver or one with no
// evaluator payload is [ErrDegenerate] (R12); a receiver owned by a
// different document is [ErrForeignBody] (R13); a retired receiver is
// [ErrRetiredBody] (R14). A failed call leaves the document and the
// receiver unchanged: every result is built before any is registered, and
// Document.commitMany registers all of them (and retires the receiver) in
// one atomic step.
func (b *Body) UnstitchContext(ctx context.Context) ([]*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an unstitch`, ErrDegenerate)
	}
	if b == nil || b.doc == nil || b.payload == nil {
		return nil, fmt.Errorf(`%w: Unstitch requires a body with an evaluator payload`, ErrDegenerate)
	}
	d := b.doc
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if _, ok := b.payload.(facetedPayload); ok {
		return nil, fmt.Errorf(`%w: Unstitch does not accept a Faceted body (docs/surface-design.md Table R row R11)`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	faces := b.Faces()
	base := d.nextProducerID()
	results := make([]*Body, len(faces))
	for i, f := range faces {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		body, err := evalUnstitchFaceContext(ctx, d, base+producerID(i), f, b.bounds, r3.Identity())
		if err != nil {
			return nil, err
		}
		results[i] = body
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commitMany(results, b)
	return results, nil
}

// unstitchPayload is Unstitch's own record: one face read straight off the
// retired receiver's own B-rep (never a previously built topology of its
// own), the rigid placement accumulated so far, and the receiver's own
// already-proven Bounds — the same "record, never rebuild" contract
// stitchPayload keeps for a whole welded set (stitch.go), restricted here to
// one face. placed() replays evalUnstitchFaceContext from these same values
// under the composed transform, exactly as stitchPayload.placed replays
// evalStitchContext.
type unstitchPayload struct {
	xform r3.Transform
	// delta is the proven displacement rigidRoundAllow charges against this
	// face's placed geometry — zero exactly when xform is the identity
	// transform, an exact struct comparison.
	delta float64
	// face is the ORIGINAL (now retired) receiver's own Face: read-only, and
	// never mutated or aliased into a live body's topology (evalUnstitchFaceContext
	// always mints a fresh copy from it).
	face *Face
	// bounds is the retired receiver's own proven, UNPLACED Bounds, read by
	// unstitchBounds (faceBounds's fallback) for every face that is not
	// straight-edged and planar — reused rather than re-derived into a
	// tighter per-face box, which would understate a curved face's true
	// extent exactly as stitchBounds's own doc comment states for a curved
	// Stitch operand (stitch.go). A single face's true box is always a
	// subset of the whole receiver's, so this stays sound even though it is
	// not the tightest box this one face could in principle earn. A
	// straight-edged planar face never reads this field: faceBounds computes
	// its own tight box off the placed face's held vertices instead.
	bounds Box
}

func (up unstitchPayload) transform() r3.Transform { return up.xform }

func (up unstitchPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	return evalUnstitchFaceContext(ctx, d, ref, up.face, up.bounds, composed)
}

// evalUnstitchFaceContext is Unstitch's whole evaluator for one face, shared
// by the initial split and every later placement: it places the face's own
// held vertices exactly once, deep-copies fresh topology under the motion so
// nothing aliases the source body, and publishes the result's measurements.
func evalUnstitchFaceContext(ctx context.Context, d *Document, ref producerID, srcFace *Face, srcBounds Box, xform r3.Transform) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	maxInputAbs := 0.0
	for _, l := range srcFace.loops {
		for _, ce := range l.coedges {
			maxInputAbs = max(maxInputAbs, vecMaxAbs(ce.edge.start.position), vecMaxAbs(ce.edge.end.position))
		}
	}
	delta := 0.0
	if xform != r3.Identity() {
		delta = rigidRoundAllow(maxInputAbs, vecMaxAbs(xform.Translation()))
	}

	nf, err := copyFaceUnderContext(ctx, srcFace, xform, delta)
	if err != nil {
		return nil, err
	}

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: false, kind: BodySheet}
	nf.body = body
	if err := attachFaceLoopsContext(ctx, []*Face{nf}); err != nil {
		return nil, err
	}
	shell := &Shell{faces: []*Face{nf}, open: shellIsOpen([]*Face{nf})}
	body.lumps = []*Lump{{shells: []*Shell{shell}}}

	// Area is the face's own reading, unaffected by delta: a rigid motion
	// preserves area exactly, and this evaluator never recomputes area from
	// placed coordinates — it copies the analytic value the source face
	// already carried (copyFaceUnderContext), so that value's own proven
	// bound already covers the placed face exactly as it covered the
	// unplaced one.
	body.area = Measurement{
		Value:     units.SquareMillimeters(nf.area),
		Exactness: exactnessOf(nf.areaBound),
		Bound:     units.SquareMillimeters(nf.areaBound),
	}

	bounds, err := faceBounds(srcFace, nf, srcBounds, xform, delta)
	if err != nil {
		return nil, err
	}
	body.bounds = bounds

	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}

	body.payload = unstitchPayload{xform: xform, delta: delta, face: srcFace, bounds: srcBounds}
	return body, nil
}

// copyFaceUnderContext deep-copies srcFace's surface, loops, edges and
// vertices under xform, minting fresh *Vertex, *Edge and *Face values that
// share no pointer with the retired receiver (docs/surface-design.md §6.5):
// a retired body stays readable, and a caller may still hold it, so mutating
// its topology through a shared pointer would corrupt what they read. It
// mirrors stitch.go's rebuildStitchTopology per-face walk with no weld plan:
// a single unstitched face never merges an edge or a vertex with anything
// else, so every new edge ends up free by construction once
// attachFaceLoopsContext registers it on this one face alone. A vertex or
// edge reached twice within the SAME face (a coedge revisiting a shared
// corner, or a full circle's own single coedge using its edge once) still
// resolves to the SAME new object, keyed by the old pointer, so the copy
// stays one connected boundary rather than disjoint free-floating pieces.
//
// Every field this copies is read straight off srcFace/its edges/vertices
// verbatim except the surface, curve and vertex position (transformed by
// xform through stitch.go's transformSurface/transformCurve) and the
// vertex bound / edge lengthBound (widened by delta under a non-identity
// placement, exactly as rebuildStitchTopology widens them). Face.axialDelta,
// Face.hasAxialDelta and Face.normalBound are left at their zero value,
// the same choice rebuildStitchTopology already makes for a stitched face:
// each is a proof tied to the face's role in the body that built it, and
// this evaluator does not re-derive it for the face's new, different body.
func copyFaceUnderContext(ctx context.Context, srcFace *Face, xform r3.Transform, delta float64) (*Face, error) {
	newVertByOld := map[*Vertex]*Vertex{}
	vertexFor := func(old *Vertex) (*Vertex, error) {
		if nv, ok := newVertByOld[old]; ok {
			return nv, nil
		}
		p := xform.Apply(old.position)
		if !finiteVec(p) {
			return nil, fmt.Errorf(`%w: a placed unstitch vertex is not representable`, ErrUnsupported)
		}
		bound := old.bound.Base()
		if delta > 0 {
			bound = absSumUpper(bound, delta)
		}
		nv := &Vertex{position: p, bound: units.Millimeters(bound)}
		newVertByOld[old] = nv
		return nv, nil
	}

	newEdgeByOld := map[*Edge]*Edge{}
	edgeFor := func(old *Edge) (*Edge, error) {
		if ne, ok := newEdgeByOld[old]; ok {
			return ne, nil
		}
		curve, err := transformCurve(old.curve, xform)
		if err != nil {
			return nil, err
		}
		start, err := vertexFor(old.start)
		if err != nil {
			return nil, err
		}
		end, err := vertexFor(old.end)
		if err != nil {
			return nil, err
		}
		lengthBound := old.lengthBound
		if delta > 0 {
			lengthBound = absSumUpper(lengthBound, delta)
		}
		ne := &Edge{
			curve:           curve,
			start:           start,
			end:             end,
			convex:          old.convex,
			length:          old.length,
			lengthBound:     lengthBound,
			lengthUnbounded: old.lengthUnbounded,
		}
		newEdgeByOld[old] = ne
		return ne, nil
	}

	surface, err := transformSurface(srcFace.surface, xform)
	if err != nil {
		return nil, err
	}
	nf := &Face{
		surface:    surface,
		origins:    append([]FeatureRef(nil), srcFace.origins...),
		area:       srcFace.area,
		areaBound:  srcFace.areaBound,
		reversed:   srcFace.reversed,
		heldPlanar: srcFace.heldPlanar,
	}
	for _, l := range srcFace.loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		coedges := make([]coedge, len(l.coedges))
		for i, ce := range l.coedges {
			ne, err := edgeFor(ce.edge)
			if err != nil {
				return nil, err
			}
			coedges[i] = coedge{edge: ne, forward: ce.forward}
		}
		nf.loops = append(nf.loops, &Loop{coedges: coedges, outer: l.outer})
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
func faceBounds(srcFace, nf *Face, srcBounds Box, xform r3.Transform, delta float64) (Box, error) {
	if srcFace.isPlanar() && faceIsStraightEdged(srcFace) {
		return facePolygonBounds(nf)
	}
	return unstitchBounds(srcBounds, xform, delta)
}

// faceIsStraightEdged reports whether every edge bounding f, across every
// loop, is a Line3 — the one Curve variant a straight-line polygon's own
// bounding box needs (see faceBounds).
func faceIsStraightEdged(f *Face) bool {
	for _, l := range f.loops {
		for _, ce := range l.coedges {
			if _, ok := ce.edge.curve.(Line3); !ok {
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
// nf holds was already placed and delta-widened by copyFaceUnderContext, so
// this needs no separate transform or delta step of its own: an unplaced,
// all-exact face therefore reads Exact with a zero bound, and a placed one
// carries exactly the placement's own rounding — never more.
func facePolygonBounds(nf *Face) (Box, error) {
	have := false
	var lo, hi r3.Vec
	maxBound := 0.0
	fold := func(v *Vertex) error {
		if !finiteVec(v.position) {
			return fmt.Errorf(`%w: a placed unstitch vertex is not representable`, ErrUnsupported)
		}
		if !have {
			lo, hi = v.position, v.position
			have = true
		} else {
			lo = r3.Vec{X: math.Min(lo.X, v.position.X), Y: math.Min(lo.Y, v.position.Y), Z: math.Min(lo.Z, v.position.Z)}
			hi = r3.Vec{X: math.Max(hi.X, v.position.X), Y: math.Max(hi.Y, v.position.Y), Z: math.Max(hi.Z, v.position.Z)}
		}
		maxBound = max(maxBound, v.bound.Base())
		return nil
	}
	for _, l := range nf.loops {
		for _, ce := range l.coedges {
			if err := fold(ce.edge.start); err != nil {
				return Box{}, err
			}
			if err := fold(ce.edge.end); err != nil {
				return Box{}, err
			}
		}
	}
	if !have {
		return Box{}, fmt.Errorf(`%w: a straight-edged face has no vertex to bound`, ErrDegenerate)
	}
	return Box{Min: lo, Max: hi, Exactness: exactnessOf(maxBound), Bound: units.Millimeters(maxBound)}, nil
}

// unstitchBounds publishes one unstitched face's box as the retiring
// receiver's own already-proven Bounds, inflated by its own Bound and then
// carried through xform by its 8 corners (stitch.go's stitchBoxCorners) —
// the same reasoning stitchBounds already states for a curved Stitch
// operand, applied here to a single face of the body being split rather
// than to a whole operand body. The published Bound is exactly delta, on
// the same terms stitchBounds publishes for a placed stitched body. It is
// faceBounds's fallback for every face that is not straight-edged and
// planar (see faceBounds's own doc comment for why the box it returns is
// sound but not necessarily tight).
func unstitchBounds(srcBounds Box, xform r3.Transform, delta float64) (Box, error) {
	inflate := srcBounds.Bound.Base()
	have := false
	var lo, hi r3.Vec
	for _, c := range stitchBoxCorners(srcBounds.Min, srcBounds.Max, inflate) {
		p := xform.Apply(c)
		if !finiteVec(p) {
			return Box{}, fmt.Errorf(`%w: a placed unstitch bound is not representable`, ErrUnsupported)
		}
		if !have {
			lo, hi = p, p
			have = true
			continue
		}
		lo = r3.Vec{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
		hi = r3.Vec{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
	}
	return Box{Min: lo, Max: hi, Exactness: exactnessOf(delta), Bound: units.Millimeters(delta)}, nil
}
