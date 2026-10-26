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
// The bevel leaves the reference face d from each edge and meets the other
// face d·tan(angle) from it. ChamferAngle states that second distance and
// builds the result with [Chamfer] and [WithAsymmetricChamfer], so the
// receivers, routes and refusals of those apply.
//
// Every selected edge MUST lie between two planar faces whose normals are
// exactly perpendicular, which a right-angled box or prism edge meets. Any
// other edge is ErrUnsupported: the tangent reading is only the second
// distance on a right-angled edge, and decad does not accept a dihedral that
// is merely close to a right angle. angle MUST be an angle strictly between 0
// and a quarter turn, and d a positive length; an angle at or past those bounds
// is ErrDegenerate, a negative value is ErrNegativeMagnitude, and a wrong unit
// kind is ErrUnitKind. reference follows
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
	if theta <= 0 || theta >= math.Pi/2 {
		return nil, fmt.Errorf(`%w: the chamfer angle must lie strictly between 0 and a quarter turn`, ErrDegenerate)
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
	for _, e := range edges {
		if err := e.requireRightAngle("a distance-and-angle chamfer"); err != nil {
			return nil, err
		}
	}
	other := units.Millimeters(dmm * math.Tan(theta))
	return Chamfer(ctx, b, sel, d, WithAsymmetricChamfer(reference, other))
}
