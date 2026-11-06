package loftmirror

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
)

// ThroughBoreCut cuts a straight through-tool from an unplaced mirrored Loft
// join (agent-docs/loft-boolean-design.md, Mirror Boolean) by its recorded
// halves: loftrecord.ThroughBoreCut bores the recorded lower Loft, the bored
// Loft is placed under the join's reflection, and the two are joined again.
// The tool must reach both outer caps of the join. It reports false, with no
// error, for any other pair, which keeps the general Boolean.
//
// The lower Loft L lies in the slab between its start plane and the mirror
// plane, and the join is L ∪ F(L) for the reflection F across that plane. A
// straight tool T along the Loft axis whose exact ends reach both outer caps
// meets that slab in the same section prism as F(T) does, so
// (L ∪ F(L)) − T = (L − T) ∪ F(L − T).
func ThroughBoreCut(ctx context.Context, op meshbool.OperationKind, env topo.Env, mesher verifiedmesh.Mesher,
	a, b *topo.Body) (*topo.Body, bool, error) {
	if op != meshbool.OpCut || mesher == nil || !a.Solid || !b.Solid {
		return nil, false, nil
	}
	fp, ok := a.Payload.(facetedrecord.Payload)
	if !ok || fp.LoftMirrorJoin == nil || fp.Xform != r3.Identity() || !fp.ContactAudited {
		return nil, false, nil
	}
	record := fp.LoftMirrorJoin
	lower, lowerOK := record.Lower.Payload.(loftrecord.Payload)
	upper, upperOK := record.Upper.Payload.(loftrecord.Payload)
	if !lowerOK || !upperOK || lower.Xform != r3.Identity() || !lower.MirrorAxialSource() {
		return nil, false, nil
	}
	reflection, err := r3.Reflection(lower.Frame1)
	if err != nil {
		return nil, true, fmt.Errorf("%w: mirror join end section has no reflection: %s", decaderr.ErrUnsupported, err)
	}
	if upper.Xform != reflection {
		return nil, false, nil
	}
	// The join's outer top cap is the start plane reflected across the end
	// plane: twice the exact gap between the two section planes above the start.
	z0 := proofarith.FloatRat(lower.Frame0.Origin().Z)
	z1 := proofarith.FloatRat(lower.Frame1.Origin().Z)
	if z0 == nil || z1 == nil {
		return nil, false, nil
	}
	span := new(big.Rat).Sub(z1, z0)
	span.Mul(span, big.NewRat(2, 1))
	if !loftrecord.ThroughToolSpans(b.Payload, lower.Frame0, span) {
		return nil, false, nil
	}
	boredLower, ok, err := loftrecord.ThroughBoreCut(ctx, env, lower, b.Payload)
	if err != nil || !ok {
		return nil, false, err
	}
	bored, ok := boredLower.Payload.(loftrecord.Payload)
	if !ok {
		return nil, true, fmt.Errorf("%w: the bored mirror half is not a loft", decaderr.ErrUnsupported)
	}
	boredUpper, err := bored.Placed(ctx, env, reflection)
	if err != nil {
		return nil, true, err
	}
	reflected, ok := boredUpper.Payload.(loftrecord.Payload)
	if !ok {
		return nil, true, fmt.Errorf("%w: the reflected bored half is not a loft", decaderr.ErrUnsupported)
	}
	body, err := buildUnion(ctx, env, mesher, boredLower, boredUpper, bored, reflected)
	return body, true, err
}
