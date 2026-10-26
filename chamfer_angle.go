package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// ChamferAngle bevels the edges sel selects by a distance d along the
// reference face and an angle between the bevel and that face, returning the
// new body and retiring b (agent-docs/chamfer-angle-design.md).
//
// The bevel leaves the reference face d from each edge and makes angle with it.
// It meets the other face d·sin(angle)/sin(φ + angle) from the edge, where φ is
// the interior angle between the two faces, which is d·tan(angle) on a right
// angle. ChamferAngle states that second distance and builds the result with
// [Chamfer] and [WithAsymmetricChamfer], so the receivers, routes and refusals of
// those apply.
//
// Every selected edge MUST be convex and lie between two planar faces. Any other
// edge is ErrUnsupported. All the selected edges MUST give the same second
// distance, within a relative 1e-12, which equal dihedrals do; edges at different
// angles take separate calls, and a selection that mixes them is ErrUnsupported.
// angle MUST be an angle strictly between 0 and a half turn, and an edge whose
// interior angle φ makes φ + angle reach a half turn has a bevel that never
// meets the other face: that is ErrDegenerate, as is an angle at or past those
// bounds. d MUST be a positive length. A negative angle or length is
// ErrNegativeMagnitude and a wrong unit kind is ErrUnitKind. reference follows
// WithAsymmetricChamfer's rule of one adjacent face per edge. A nil context,
// body or selector is ErrDegenerate. Every failure leaves b live and the
// document unchanged.
func ChamferAngle(ctx context.Context, b *Body, sel EdgeSelector, d units.Value, reference FaceSelector,
	angle units.Value) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationChamfer, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a chamfer`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	if reference == nil {
		return nil, errNilSelector
	}
	theta, err := sectionrecord.MagnitudeIn(angle, units.Angle, units.Radian, "the chamfer angle")
	if err != nil {
		return nil, err
	}
	if theta <= 0 || theta >= math.Pi {
		return nil, fmt.Errorf(`%w: the chamfer angle must lie strictly between 0 and a half turn`, ErrDegenerate)
	}
	dmm, err := sectionrecord.MagnitudeIn(d, units.Length, units.Millimeter, "the chamfer distance")
	if err != nil {
		return nil, err
	}
	// A selector Chamfer would refuse is left to Chamfer, so the error is its own.
	q, ok := sel.(*EdgeQuery)
	if !ok || q == nil {
		return Chamfer(ctx, b, sel, d)
	}
	edges, err := q.SelectEdges(b)
	if err != nil {
		return nil, err
	}
	var second float64
	for i, e := range edges {
		phi, err := e.convexDihedral("a distance-and-angle chamfer")
		if err != nil {
			return nil, err
		}
		if phi+theta >= math.Pi {
			return nil, fmt.Errorf(`%w: a bevel at %g radians to a face meeting the other at %g radians never meets it`,
				ErrDegenerate, theta, phi)
		}
		other := dmm * math.Sin(theta) / math.Sin(phi+theta)
		if i == 0 {
			second = other
		} else if !sameWithinDihedralTolerance(second, other) {
			return nil, fmt.Errorf(`%w: the selected edges meet at different angles, so they need different second distances`, ErrUnsupported)
		}
	}
	return Chamfer(ctx, b, sel, d, WithAsymmetricChamfer(reference, units.Millimeters(second)))
}
