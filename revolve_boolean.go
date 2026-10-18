package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
)

// tryCommonAxisRevolveBoolean keeps the public body and error boundary after
// the recorded meridians have been admitted by revolveplan.
func tryCommonAxisRevolveBoolean(ctx context.Context, op meshbool.OperationKind, d *Document,
	ref producerID, a, b *Body) (*Body, bool, error) {
	result, admitted, empty, err := revolveplan.PrepareCommonAxis(ctx, op, &a.n, &b.n)
	if err != nil || !admitted {
		return nil, false, err
	}
	if empty {
		return nil, true, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedEmpty,
			fmt.Errorf("%w: the common-axis result contains no volume", ErrBooleanFailed)))
	}
	body, err := bodyResult(revolveplan.Build(ctx, envOf(documentNode(d), ref), result, freeform.NewFreeformWork()))
	if err != nil {
		return nil, false, err
	}
	return body, true, nil
}
