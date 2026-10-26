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
// Where the faces meet at the angle φ across the corner (the interior angle of a
// convex edge, the open wedge of a concave one) the arc turns π − φ, so its
// radius is chord/(2·cos(φ/2)), which is chord/√2 on a right angle.
// FilletChord states that radius and builds the result with [Fillet], so the
// receivers, routes and refusals of Fillet apply.
//
// Every selected edge MUST lie between two planar faces, and may be convex or
// concave: a concave edge's fillet adds the round's material to the corner. Any
// other edge is ErrUnsupported. All the selected edges MUST give the same radius,
// within a relative 1e-12, which equal dihedrals do; edges at different angles
// take separate calls, and a selection that mixes them is ErrUnsupported. chord
// MUST be a positive length; a zero or negative one is ErrDegenerate or
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
	var radius float64
	for i, e := range edges {
		phi, err := e.planarDihedral("a chord-length fillet")
		if err != nil {
			return nil, err
		}
		r := cmm / (2 * math.Cos(phi/2))
		if i == 0 {
			radius = r
		} else if !sameWithinDihedralTolerance(radius, r) {
			return nil, fmt.Errorf(`%w: the selected edges meet at different angles, so they need different radii`, ErrUnsupported)
		}
	}
	return Fillet(ctx, b, sel, units.Millimeters(radius))
}
