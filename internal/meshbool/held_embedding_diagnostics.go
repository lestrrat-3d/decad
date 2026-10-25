package meshbool

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// EmbeddingReach describes final positions accepted from the positive-reach
// search, excluding float-box-only repair and inherited geometry bounds.
type EmbeddingReach struct {
	DisplacementBound float64
	AllowedReach      float64
}

// Diagnostics returns the bounded result-change record, or nil without a repair.
// The public operation boundary supplies the operation kind before publication.
func (r *EmbeddingReach) Diagnostics() []opdiagnostic.Record {
	if r == nil {
		return nil
	}
	return []opdiagnostic.Record{{
		Code: opdiagnostic.CodeEmbeddingReachApplied, Operand: -1,
		Change:            opdiagnostic.ChangeEmbeddingReach,
		DisplacementBound: r.DisplacementBound, AllowedReach: r.AllowedReach,
	}}
}

// EnforceHeldEmbeddingWithSummary is EnforceHeldEmbedding with actual-use
// evidence for final accepted positive-reach positions. A refused or canceled
// repair returns no summary, even when some trial positions were improved.
func EnforceHeldEmbeddingWithSummary(ctx context.Context, h HeldRounding) (int, *EmbeddingReach, error) {
	return enforceHeldEmbeddingWithSummary(ctx, h, true)
}

// RepairHeldEmbeddingWithSummary is RepairHeldEmbedding with actual-use
// evidence only when no folded facets remain.
func RepairHeldEmbeddingWithSummary(ctx context.Context, h HeldRounding) (int, []int, *EmbeddingReach, error) {
	var summary EmbeddingReach
	moved, failing, err := repairHeldEmbeddingSummary(ctx, h, true, &summary)
	if err != nil || len(failing) != 0 || summary.DisplacementBound == 0 {
		return moved, failing, nil, err
	}
	return moved, failing, &summary, nil
}

// summarizeReach reads only vertices whose current position was accepted from
// improveWithinReach. A later accepted float-box corner removes that vertex.
func (e *heldEmbedding) summarizeReach(ctx context.Context, summary *EmbeddingReach) error {
	if summary == nil || len(e.reachUsed) == 0 {
		return nil
	}
	var bound float64
	i := 0
	for v := range e.reachUsed {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		i++
		gap := CoordDistance(e.h.Exact[v], e.h.Verts[v])
		if gap.Sign() == 0 {
			continue
		}
		w, _ := gap.Float64()
		bound = math.Max(bound, proofbound.Radius3D(proofbound.ProvenUpRound(w)))
	}
	if bound > 0 {
		*summary = EmbeddingReach{DisplacementBound: bound, AllowedReach: e.h.Reach}
	}
	return nil
}
