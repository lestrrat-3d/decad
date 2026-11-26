package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

// IntersectionVolume runs the evaluation Intersect runs on a and b, in that operand order, and returns the volume
// of the body Intersect would build. It commits nothing: Bodies, the operation history, producer numbering,
// containers and every body are unchanged, and both operands stay live. It may advance the private level and
// curve identity counters, as a refused Intersect does.
//
// The answer is Intersect's, bit for bit: the Measurement equals the Volume of the body Intersect returns for
// the same operands in the same document state, and every error is the error Intersect returns, tagged
// OperationIntersect with the same diagnostics. An empty intersection is the *BooleanError with Code BooleanEmpty, carrying the gap
// shift Intersect reports, if one applied. The one difference is the container rule: a and b may sit in different
// containers (agent-docs/container-design.md §3).
//
// A nil ctx, a nil body and a == b are ErrDegenerate; a body of another document is ErrForeignBody; a retired
// body is ErrRetiredBody. A ctx canceled during the evaluation returns ctx.Err() unchanged. It may run
// concurrently with Bodies, Verify, the body readers and other IntersectionVolume calls on the same document,
// never with an operation.
func (d *Document) IntersectionVolume(ctx context.Context, a, b *Body) (Measurement, error) {
	m, err := d.intersectionVolume(ctx, a, b)
	return m, operationError(OperationIntersect, err)
}

func (d *Document) intersectionVolume(ctx context.Context, a, b *Body) (Measurement, error) {
	if ctx == nil {
		return Measurement{}, fmt.Errorf(`%w: a nil context cannot control a boolean`, ErrDegenerate)
	}
	if err := d.requireLive(a); err != nil {
		return Measurement{}, err
	}
	if err := d.requireLive(b); err != nil {
		return Measurement{}, err
	}
	if a == b {
		return Measurement{}, fmt.Errorf(`%w: a boolean needs two distinct bodies`, ErrDegenerate)
	}
	build, err := buildBoolean(ctx, d, meshbool.OpIntersect, a, b, booleanConfig{})
	if err != nil {
		return Measurement{}, err
	}
	return build.body.Volume()
}
