package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// FilletChord rounds the edges sel selects with an arc whose chord, the
// straight line between its two tangent edges, is chord long, returning the
// new body and retiring b (agent-docs/fillet-chord-design.md).
//
// On a right-angled edge the arc turns a quarter turn, so its radius is
// chord/√2. FilletChord states that radius and builds the result with
// [Fillet], so the receivers, routes and refusals of Fillet apply.
//
// Every selected edge MUST lie between two planar faces whose normals are
// exactly perpendicular, which a right-angled box or prism edge meets. Any
// other edge is ErrUnsupported: the radius is chord/√2 only at a right angle,
// and decad does not accept a dihedral that is merely close to one. chord MUST
// be a positive length; a zero or negative one is ErrDegenerate or
// ErrNegativeMagnitude, and a wrong unit kind is ErrUnitKind. A nil context,
// body or selector is ErrDegenerate. Every failure leaves b live and the
// document unchanged.
func FilletChord(ctx context.Context, b *Body, sel EdgeSelector, chord units.Value) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationFillet, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a fillet`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	cmm, err := sectionrecord.MagnitudeIn(chord, units.Length, units.Millimeter, "the fillet chord")
	if err != nil {
		return nil, err
	}
	if cmm == 0 {
		return nil, fmt.Errorf(`%w: a zero-chord fillet is the body the caller already holds`, ErrDegenerate)
	}
	// A selector Fillet would refuse is left to Fillet, so the error is its own.
	q, ok := sel.(*EdgeQuery)
	if !ok || q == nil {
		return Fillet(ctx, b, sel, chord)
	}
	edges, err := q.SelectEdges(b)
	if err != nil {
		return nil, err
	}
	for _, e := range edges {
		if err := e.requireRightAngle("a chord-length fillet"); err != nil {
			return nil, err
		}
	}
	return Fillet(ctx, b, sel, units.Millimeters(cmm/math.Sqrt2))
}
