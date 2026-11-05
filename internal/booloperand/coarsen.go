package booloperand

import (
	"errors"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// MaxOperandCoarsenings is how many times a revolve operand's chord request may
// be doubled past the pair tolerance, so it never exceeds 2^3 = 8 times it. The
// cap is fixed; no caller sets it (agent-docs/revolve-loft-design.md, Operand coarsening §2).
const MaxOperandCoarsenings = 3

// CoarsensOperand reports whether err, the refusal of meshing b at its current
// chord request, is one a coarser request can answer: a revolve operand whose
// mesh ran into a work ceiling, the facet ceiling of one mesh or the
// facet-contact audit's pair ceiling. Both ceilings grow as the chord shrinks.
// Every other refusal, and every other payload, is final.
//
// A nil err asks only whether b's payload is one the ladder applies to.
func CoarsensOperand(b *topo.Body, err error) bool {
	if _, ok := b.Payload.(revolveplan.Payload); !ok {
		return false
	}
	if err == nil {
		return true
	}
	var limit *proofbound.WorkLimitError
	return errors.As(err, &limit)
}
