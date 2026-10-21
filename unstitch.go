package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file is Unstitch of agent-docs/surface-design.md §6.5: the inverse of
// stitch.go's Stitch, splitting a body back into one single-face sheet body
// per face. Its evaluator is internal/stitchrecord's BuildUnstitchFace.
// Document.commitMany (document.go) is the multi-produce atomic commit this
// file's N results need beside the existing single-body commit.

// Unstitch returns one single-face sheet body per face of
// b, in [Body.Faces] order, retiring b
// (agent-docs/surface-design.md §6.5). Each result carries that face's own
// surface, loops and readings, and every edge of every result is free: a
// fresh Face, Edge and Vertex set is minted for each result rather than
// reusing b's own topology, so the retired b stays
// readable and unmodified for any caller still holding it.
//
// It admits an analytic body, solid or sheet — the asymmetry with [Stitch],
// which takes sheets only (Table R row R17), is deliberate: Unstitch is how
// a caller gets sheets to work with in the first place. A Faceted body — a
// mesh boolean's result — is [ErrUnsupported] (R11): its faces are chord
// polygons with no analytic identity, and returning thousands of facet
// sheets would be a shape nobody asked for. A nil b or a b with no
// evaluator payload is [ErrDegenerate] (R12); a b owned by a
// different document is [ErrForeignBody] (R13); a retired b is
// [ErrRetiredBody] (R14). A failed call leaves the document and
// b unchanged: every result is built before any is registered, and
// Document.commitMany registers all of them (and retires b) in
// one atomic step.
func Unstitch(ctx context.Context, b *Body) ([]*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an unstitch`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil || b.n.Payload == nil {
		return nil, fmt.Errorf(`%w: Unstitch requires a body with an evaluator payload`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if _, ok := b.n.Payload.(facetedPayload); ok {
		return nil, fmt.Errorf(`%w: Unstitch does not accept a Faceted body (agent-docs/surface-design.md Table R row R11)`, ErrUnsupported)
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
		assertUnstitchFaceOwners(faceNode(f), r3.Identity())
		body, err := bodyResult(stitchrecord.BuildUnstitchFace(ctx, envOf(documentNode(d), base+producerID(i)), faceNode(f), b.n.Bounds, r3.Identity()))
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

// assertUnstitchFaceOwners maps every edge and vertex node of srcFace that
// Unstitch's face copy reads back to its root handle, in the copy's order, so
// a node whose Owner is not its handle fails the mapping's assertion as it
// always has. It checks nothing when the copy refuses first (a placed face
// with a nonzero normalBound) and stops where the build would dereference a
// nil face, loop, edge or vertex, leaving that panic to
// stitchrecord.BuildUnstitchFace.
func assertUnstitchFaceOwners(srcFace *topo.Face, xform r3.Transform) {
	if srcFace == nil {
		return
	}
	for _, l := range srcFace.Loops {
		if l == nil {
			return
		}
		for _, ce := range l.CoEdges {
			if ce.Edge == nil || ce.Edge.Start == nil || ce.Edge.End == nil {
				return
			}
		}
	}
	if xform != r3.Identity() && srcFace.NormalBound != 0 {
		return
	}
	for _, l := range srcFace.Loops {
		for _, ce := range l.CoEdges {
			edgeHandle(ce.Edge)
			vertexHandle(ce.Edge.Start)
			vertexHandle(ce.Edge.End)
		}
	}
}
