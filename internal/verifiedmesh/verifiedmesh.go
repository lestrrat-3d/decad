// Package verifiedmesh names the one tessellation an internal builder borrows
// from the root: a body's mesh at the full verification level.
package verifiedmesh

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/topo"
)

// Mesher tessellates a body at VerifyAll with chord tolerance tolMM, so the
// returned mesh carries every proof a tessellation can (agent-docs/tessellation-
// design.md §2).
type Mesher interface {
	VerifyAll(ctx context.Context, b *topo.Body, tolMM float64) (*topo.Mesh, error)
}
