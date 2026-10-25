package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/mirrorjoin"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file adapts internal/mirrorjoin's record rewrite and joined-section
// audit to prism and stacked payloads.

type mirrorJoinOption struct{}

func (mirrorJoinOption) mirrorOption() {}

// WithJoin makes [Mirrored] return the union of the receiver and its
// mirror image as one body: the symmetric-part operation
// (agent-docs/mirror-pattern-design.md §5). The plane must be a [MirrorFace] of the
// receiver itself naming one or more planar walls on one line of its section;
// the selector then asserts at least one face rather than exactly one.
//
// The join rewrites the receiver's own recorded section and never runs a
// boolean. It admits a straight prism or a stacked prism (a blind-cut plate)
// whose every other boundary segment is a whole line, a whole arc or a whole
// circle lying on the receiver's side of the mirror line, and whose recorded
// section carries no displacement. Every other receiver is refused with the
// sentinel §5.1 states — ErrUnsupported for a shape the join does not build,
// ErrDegenerate for a selection that names no single mirror line — and is
// never handed to Union instead. A repeated WithJoin() is idempotent.
func WithJoin() MirrorOption {
	return mirrorJoinOption{}
}

// mirroredJoin is Mirrored under WithJoin. It runs Transform's receiver gates,
// then §5.1's admission J1-J7 in order, assembles the joined record (§5.2),
// audits it (§5.3) and commits the result in place of the receiver.
func (b *Body) mirroredJoin(ctx context.Context, plane MirrorPlane) (*Body, error) {
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.n.Payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot mirror a body it did not build`, ErrUnsupported)
	}
	// J1: a straight prism or a stacked prism, and a solid.
	switch b.n.Payload.(type) {
	case prismPayload, stackedPrismPayload:
	default:
		return nil, fmt.Errorf(`%w: a mirror join rewrites a prism's or a stacked prism's section; join any other body with MirroredCopy and Union`, ErrUnsupported)
	}
	if b.Kind() == BodySheet {
		return nil, fmt.Errorf(`%w: a mirror join builds a solid and does not accept a sheet body`, ErrUnsupported)
	}
	if sp, ok := b.n.Payload.(stackedPrismPayload); ok && sp.IsGroup() {
		// A prism group's lumps are separate regions, which this join does
		// not splice one by one.
		return nil, fmt.Errorf(`%w: a mirror join does not rewrite a prism group (J1)`, ErrUnsupported)
	}
	walls, err := d.resolveJoinWalls(b, plane)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	ref := d.nextProducerID()
	var body *Body
	switch p := b.n.Payload.(type) {
	case prismPayload:
		joined, err := mirrorjoin.JoinPrismPayload(budget, p, walls)
		if err != nil {
			return nil, err
		}
		body, err = bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), joined, freeform.NewFreeformWork()))
		if err != nil {
			return nil, err
		}
	case stackedPrismPayload:
		joined, err := mirrorjoin.JoinStackedPayload(ctx, budget, p, walls)
		if err != nil {
			return nil, err
		}
		body, err = bodyResult(stackedrecord.Build(ctx, envOf(documentNode(d), ref), joined, stackedrecord.OwnPlan{}))
		if err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := d.commitOperationIn(ctx, nil, OperationMirrored,
		[]OperationInput{{Body: b, Role: operationRoleSource, Use: InputConsumed},
			{Body: b, Role: "plane", Use: InputReferenced}}, body, b); err != nil {
		return nil, err
	}
	return body, nil
}

// resolveJoinWalls is J2's selection half: the plane must be a MirrorFace of
// the receiver, its selector resolves to at least one face, and every face
// must be a wall the receiver's own producer minted a side role for. Whether
// that wall is planar is read off the record by the payload's own join.
func (d *Document) resolveJoinWalls(b *Body, plane MirrorPlane) ([]mirrorjoin.WallRef, error) {
	var mf MirrorFace
	switch p := plane.(type) {
	case MirrorFace:
		mf = p
	case *MirrorFace:
		if p == nil {
			return nil, errNilMirrorPlane
		}
		mf = *p
	case *MirrorFrame:
		if p == nil {
			return nil, errNilMirrorPlane
		}
		return nil, errJoinNeedsFace
	case MirrorFrame:
		return nil, errJoinNeedsFace
	default:
		return nil, errNilMirrorPlane
	}
	if mf.Body == nil {
		return nil, fmt.Errorf(`%w: a mirror face names no body to resolve against`, ErrDegenerate)
	}
	if err := d.requireLive(mf.Body); err != nil {
		return nil, err
	}
	if mf.Body != b {
		return nil, fmt.Errorf(`%w: a mirror join needs its mirror line in the receiver's own record, so the mirror face must be a face of the receiver`, ErrUnsupported)
	}
	faces, err := selectAtLeastOneFace(b, mf.Face, "the mirror face")
	if err != nil {
		return nil, err
	}
	_, stacked := b.n.Payload.(stackedPrismPayload)
	producer := b.originProducer()
	var walls []mirrorjoin.WallRef
	for fi, f := range faces {
		found := false
		for _, o := range f.n.Origins {
			if o.Producer != producer {
				continue
			}
			var k, i, j int
			if n, _ := fmt.Sscanf(o.Role, "slab(%d).region(0).side(%d,%d)", &k, &i, &j); n == 3 && stacked {
				walls = append(walls, mirrorjoin.WallRef{Slab: k, Loop: i, Segment: j})
				found = true
				continue
			}
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &i, &j); n == 2 && !stacked {
				walls = append(walls, mirrorjoin.WallRef{Slab: -1, Loop: i, Segment: j})
				found = true
			}
		}
		if !found {
			return nil, fmt.Errorf(`%w: selected face %d is not a wall of the receiver (a cap or a floor states no mirror line in the section)`, ErrDegenerate, fi)
		}
	}
	return walls, nil
}

// errJoinNeedsFace is J2's refusal of a MirrorFrame: the join reads its
// mirror line from a recorded wall, which a caller-built frame does not name.
var errJoinNeedsFace = fmt.Errorf(`%w: a mirror join needs the mirror line as a recorded wall; name it with a MirrorFace of the receiver, not a MirrorFrame`, ErrUnsupported)

// selectAtLeastOneFace resolves sel against body asserting at least one
// match: an unasserted zero is ErrCardinality with Expected "at least 1",
// and a failed assertion of the selector's own is returned unchanged.
func selectAtLeastOneFace(body *Body, sel FaceSelector, what string) ([]*Face, error) {
	q, err := builtinFaceQuery(sel, what)
	if err != nil {
		return nil, err
	}
	faces, err := q.SelectFaces(body)
	if err != nil {
		if errors.Is(err, ErrCardinality) {
			return nil, err
		}
		if errors.Is(err, ErrNoMatch) {
			return nil, q.selectionError(body, 0, "at least 1", ErrCardinality)
		}
		return nil, err
	}
	return faces, nil
}
