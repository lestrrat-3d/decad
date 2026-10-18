package decad

import (
	"context"
	"fmt"
	"iter"

	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"

	"github.com/lestrrat-3d/r3"
)

// This file is Stitch of docs/surface-design.md §6: the one operation that
// turns a boundary into a solid. Its evaluator is internal/stitchrecord's
// Build.

// Stitch welds every free-edge pair it can prove coincident
// (docs/surface-design.md §6.2's Table J) across bodies, assembles the
// result, and returns one new body, retiring every operand. One operand is
// meaningful: it re-audits that body's own boundary, which is how a closed
// sheet becomes a solid (§2.1, §4.1).
//
// Every operand must carry an evaluator payload and be a live [BodySheet] of
// this call's document: a nil or payload-less operand is [ErrDegenerate]
// (Table R row R12), an operand owned by a different document is
// [ErrForeignBody] (R13), a retired operand is [ErrRetiredBody] (R14), and a
// [BodySolid] operand is [ErrUnsupported] (R17) — it has no free edge, so
// stitching cannot change it, and a caller combining two solids means
// [Union]. Before Table C decides anything, every vertex's own link across
// the welded set must be a manifold — one connected fan (interior) or path
// (rim) — open or closed, all-planar or curved alike; a pinch there is
// [ErrDegenerate] (R7), the identical sentinel a non-orientable assembly
// gets, even when free edges also remain. Table C then decides the rest: a
// residual free edge is a [BodySheet] and never an error; a fully welded,
// all-planar, non-crossing boundary is a [BodySolid]; a fully welded
// boundary holding a curved face is [ErrUnsupported] (R8); a welded set
// with no consistent orientation is [ErrDegenerate] (R7); and a proven
// self-contact or an exhausted audit budget surfaces the crossing audit's
// own [ErrDegenerate]/[ErrUnsupported] (R9/R10) unchanged. A failed call
// leaves the document and every operand unchanged.
func Stitch(ctx context.Context, bodies ...*Body) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a stitch`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(bodies) == 0 {
		return nil, fmt.Errorf(`%w: Stitch requires at least one body`, ErrDegenerate)
	}
	for _, b := range bodies {
		if b == nil || b.n.Payload == nil {
			return nil, fmt.Errorf(`%w: Stitch requires every operand to carry an evaluator payload`, ErrDegenerate)
		}
	}
	d := documentHandle(bodies[0].n.Doc)
	for _, b := range bodies {
		if err := d.requireLive(b); err != nil {
			return nil, err
		}
	}
	for _, b := range bodies {
		if b.Kind() == BodySolid {
			return nil, fmt.Errorf(`%w: Stitch does not accept a solid body operand (docs/surface-design.md Table R row R17)`, ErrUnsupported)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	var srcFaces []*topo.Face
	for _, b := range bodies {
		faces := topobuild.Faces(&b.n)
		for _, f := range faces {
			faceHandle(f)
		}
		srcFaces = append(srcFaces, faces...)
	}
	assertStitchSourceOwners(srcFaces)
	plan := stitchrecord.BuildPlan(srcFaces)

	ref := d.nextProducerID()
	body, err := bodyResult(stitchrecord.Build(ctx, envOf(documentNode(d), ref), srcFaces, plan, r3.Identity()))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, bodies...)
	return body, nil
}

// assertStitchSourceOwners maps every operand vertex node, then every operand
// edge node, that Stitch's weld plan and topology rebuild read back to its
// root handle, so a node whose Owner is not its handle fails the mapping's
// assertion as it always has. It stops where the weld plan would dereference
// a nil edge or vertex, leaving that panic to stitchrecord.BuildPlan. The edge
// checks run here, ahead of the build's own context poll and placement and
// surface refusals.
func assertStitchSourceOwners(faces []*topo.Face) {
	for _, f := range faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				start := topobuild.CoEdgeStart(ce)
				if start == nil {
					return
				}
				vertexHandle(start)
				end := topobuild.CoEdgeEnd(ce)
				if end == nil {
					return
				}
				vertexHandle(end)
			}
		}
	}
	for _, f := range faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				edgeHandle(ce.Edge)
			}
		}
	}
}

// assertPlacedStitchOwners maps every operand edge and vertex node a stitch
// payload's placement rebuild reads back to its root handle, in that rebuild's
// order. It checks nothing when the rebuild refuses first (a placed operand
// with a nonzero normalBound) and stops where the rebuild would dereference a
// nil face, edge or vertex.
func assertPlacedStitchOwners(faces []*topo.Face, xform r3.Transform) {
	if xform != r3.Identity() {
		for _, f := range faces {
			if f == nil {
				return
			}
			if f.NormalBound != 0 {
				return
			}
		}
	}
	for _, f := range faces {
		if f == nil {
			return
		}
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				edgeHandle(ce.Edge)
				if ce.Edge == nil || ce.Edge.Start == nil {
					return
				}
				vertexHandle(ce.Edge.Start)
				if ce.Edge.End == nil {
					return
				}
				vertexHandle(ce.Edge.End)
			}
		}
	}
}

// stitchVertexBounds yields the proven bound magnitude of each of b's own
// vertices, in b.Vertices() order. clearance_geom.go's addStitchFaces dispatch
// gate (docs/clearance-design.md §2) and tessellate_stitch.go's occupied-volume
// admission (docs/tessellation-design.md §2's stitchPayload row) both reduce
// it through stitchweld.ZeroBound, and clearance_geom.go reads the largest
// through stitchweld.MaxBound as the body's own bodyGeom.delta once a bounded
// stitched body earns a model rather than a refusal: every vertex this carrier
// model reads comes straight off the body's own topology, so the worst vertex
// bound is a sound charge against the whole model, exactly as a placed prism's
// frame/placement rounding is (bodyGeom's own doc comment).
func stitchVertexBounds(b *Body) iter.Seq[float64] {
	return func(yield func(float64) bool) {
		for _, v := range b.Vertices() {
			if !yield(v.Position().Bound.Mag()) {
				return
			}
		}
	}
}
