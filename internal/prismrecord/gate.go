package prismrecord

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// RequireExactSection refuses a modify receiver whose payload carries a section
// displacement (agent-docs/prism-boolean-design.md §7). Every modify op is a rewrite
// of the receiver's recorded section (modify §2), and a rewrite of a record that
// is only within a displacement of the section it denotes has no proven
// displacement of its own — an offset miter or a blend centre amplifies it by
// the corner geometry, which nothing here bounds. The body exists and this
// evaluator cannot build it, so the sentinel is ErrUnsupported (modify §1's
// existence test). Zero for every payload a caller draws, so this changes
// nothing for a drawn, placed or already-modified prism.
func (pp Payload) RequireExactSection(op string) error {
	if pp.SectionDelta == 0 {
		return nil
	}
	return fmt.Errorf(
		`%w: this evaluator %s a receiver whose recorded section is the section it denotes only; this body's section carries a proven displacement of %g mm from the one its construction denotes`,
		decaderr.ErrUnsupported, op, pp.SectionDelta,
	)
}
