package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
)

// This file is the root's side of docs/brep-modify-design.md's receiver
// dispatch: the cap faces a modify op classifies a prism selection against,
// the commit of a modify op's result, and the conversions between the
// handle-level classification code and internal/brepmodify's node-level
// routes, which own the dispatch itself.

// prismCaps names the two cap faces a modify op classifies a prism selection
// against (brep-modify §4.2): start at the prism's z0 level, end at its z1
// level. startLoop and endLoop map a cap face's loop index to the section
// loop it bounds (0 the outer, 1+i hole i); nil reads as the identity. A nil
// face names no cap, and a selection on it classifies as no cap edge.
type prismCaps struct {
	start, end         *Face
	startLoop, endLoop []int
}

// prismCapsOf names a prism body's own capStart and capEnd faces.
func prismCapsOf(b *Body) prismCaps {
	roles := facesByRole(b)
	return prismCaps{start: roles[provenance.RoleCapStart], end: roles[provenance.RoleCapEnd]}
}

// sectionLoop maps one cap face's loop index to its section loop index.
func (c prismCaps) sectionLoop(start bool, li int) int {
	return c.nodes().SectionLoop(start, li)
}

// nodes is c as the node-level caps internal/brepmodify reads.
func (c prismCaps) nodes() brepmodify.PrismCaps {
	return brepmodify.PrismCaps{Start: faceNode(c.start), End: faceNode(c.end), StartLoop: c.startLoop, EndLoop: c.endLoop}
}

// prismCapsOfNodes is caps as the handle-level caps the classification code reads.
func prismCapsOfNodes(c brepmodify.PrismCaps) prismCaps {
	return prismCaps{start: faceHandle(c.Start), end: faceHandle(c.End), startLoop: c.StartLoop, endLoop: c.EndLoop}
}

// publicCurveName names a curve as the public API does, for the refusal that
// prints it.
func publicCurveName(c surfacegeom.Curve) string { return fmt.Sprintf("%T", publicCurve(c)) }

// commitModifyResult commits a modify op's result in place of its receiver,
// after the receiver's liveness and the context are read once more at the
// commit edge (modify §13's atomic commit).
func commitModifyResult(ctx context.Context, b, body *Body) (*Body, error) {
	if err := documentHandle(b.n.Doc).requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	documentHandle(b.n.Doc).commit(body, b)
	return body, nil
}
