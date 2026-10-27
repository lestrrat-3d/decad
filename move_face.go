package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/units"
)

// MoveFace moves one face of b along its normal by distance and returns the new body, retiring b: the
// press-pull of a CAD system (agent-docs/orthogonal-move-design.md). A positive distance moves the
// face outward, growing the body, and a negative one inward, cutting it, and the walls beside the face
// follow it. The result is the body joined to the prism the face sweeps, or less it, so a notch
// bottom moves as readily as an outer wall.
//
// The route is the grid one of the orthogonal shell (agent-docs/orthogonal-shell-design.md): b
// MUST be a faceted solid, such as one made by [SolidFromMesh], every face of which squares to a
// world axis and whose held vertices carry no bound, otherwise ErrUnsupported. sel MUST resolve to
// exactly one planar face of b, otherwise ErrDegenerate. distance is a signed length: zero is
// ErrDegenerate and a wrong unit kind ErrUnitKind. A face that would move into material, or inward
// out through the opposite wall, or leave no solid, is ErrDegenerate. The new coordinate is the
// float sum of the face's plane and distance. A nil context, body or selector is ErrDegenerate. The
// result is a mesh solid, so its axis faces keep their exact planes.
func MoveFace(ctx context.Context, b *Body, sel FaceSelector, distance units.Value) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationMoveFace, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a face move`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	if sel == nil {
		return nil, errNilSelector
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: MoveFace needs a solid body`, ErrNotSolid)
	}
	if distance.Kind() != units.Length {
		return nil, fmt.Errorf(`%w: the face move must be a length`, ErrUnitKind)
	}
	if base := distance.Base(); math.IsNaN(base) || math.IsInf(base, 0) {
		return nil, fmt.Errorf(`%w: the face move is not finite`, ErrNotFinite)
	}
	mm, err := distance.In(units.Millimeter)
	if err != nil {
		return nil, fmt.Errorf(`%w: the face move must be a length: %s`, ErrUnitKind, err)
	}
	if mm == 0 {
		return nil, fmt.Errorf(`%w: a zero face move changes nothing`, ErrDegenerate)
	}
	faces, err := sel.SelectFaces(b)
	if err != nil {
		return nil, err
	}
	if len(faces) != 1 {
		return nil, fmt.Errorf(`%w: MoveFace needs exactly one face, the selector matched %d`, ErrDegenerate, len(faces))
	}
	payload, ok := b.n.Payload.(facetedPayload)
	if !ok || len(payload.Tris) == 0 {
		return nil, fmt.Errorf(`%w: MoveFace takes a faceted solid whose faces all square to an axis, such as a mesh solid`, ErrUnsupported)
	}
	for _, beta := range payload.VertexBound {
		if beta != 0 {
			return nil, fmt.Errorf(`%w: MoveFace takes a faceted solid whose held vertices carry no bound`, ErrUnsupported)
		}
	}
	marked := faceTriangles(b, payload, faceNodes(faces))
	verts, tris, err := orthoshell.Press(payload.Verts, payload.Tris, marked, mm)
	switch {
	case errors.Is(err, orthoshell.ErrNotOrthogonal):
		return nil, fmt.Errorf(`%w: MoveFace takes a solid whose faces all square to an axis: %s`, ErrUnsupported, err)
	case errors.Is(err, orthoshell.ErrDegenerate):
		return nil, fmt.Errorf("%w: %s", ErrDegenerate, err)
	case errors.Is(err, orthoshell.ErrUnsupported):
		return nil, fmt.Errorf("%w: %s", ErrUnsupported, err)
	case err != nil:
		return nil, err
	}
	mesh, err := NewMeshBody(ctx, verts, tris)
	if err != nil {
		return nil, err
	}
	moved, err := solidFromMeshBody(ctx, d, mesh)
	if err != nil {
		return nil, fmt.Errorf("%w: the moved solid is not one the evaluator can prove: %s", ErrUnsupported, err)
	}
	return commitModifyResult(ctx, OperationMoveFace, b, moved)
}
