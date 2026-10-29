// Package closedstep bridges authenticated complete surfaces to the STEP writer.
package closedstep

import (
	"context"

	"github.com/lestrrat-3d/r3"
)

// Coverage contains call-local, exactly represented complete-surface data.
type Coverage struct {
	Center          r3.Vec
	Radius          float64
	Axis, Reference r3.Vec
	Major, Minor    float64
	Ring            bool
}

// Reader is installed by the root package to authenticate private source data.
var Reader func(context.Context, any, any) (Coverage, bool, error)

// Read authenticates a complete surface, or declines when no reader is installed.
func Read(ctx context.Context, body, face any) (Coverage, bool, error) {
	if err := ctx.Err(); err != nil {
		return Coverage{}, false, err
	}
	if Reader == nil {
		return Coverage{}, false, nil
	}
	return Reader(ctx, body, face)
}
