package freeform

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// RequireFiniteFreeformRange rejects a non-finite recorded range on ANY
// free-form kind. Core §12 gives [ErrNotFinite] for a non-finite input, which is
// what every other segment kind reports for exactly this field and what the
// free-form path already reports for a non-finite control coordinate. It is
// therefore decided ahead of the kind dispatch, never inside one kind's arm: a
// NaN fails both full-domain equality tests, so a kind that tested the range
// alone would report it as a trimmed range and a kind that never tested it at
// all would report its own staging reason instead. The test is O(1) on the
// recorded parameters, so it stands with the structural size refusals, ahead of
// any content scan.
func RequireFiniteFreeformRange(tStart, tEnd float64, what string) error {
	if FiniteMomentValues(tStart, tEnd) {
		return nil
	}
	return fmt.Errorf(
		`%w: a %s's recorded range is not finite (range [%v, %v])`,
		decaderr.ErrNotFinite, what, tStart, tEnd,
	)
}

// RequireFullFreeformRange rejects a recorded free-form range that is not the
// entity's full domain. spline design §2 proves none is recordable, so reaching
// this is a caller-built or decoded record that bypassed the seam — refuse
// rather than integrate a piece the conversion does not cover.
//
// It is the Tier A arms' own gate, and it stays there. Table R states R2
// unconditionally and carries no row for a trimmed range reaching the evaluator,
// so a kind refused for its own cause reports that cause whatever its range
// says. A FitSplineSeg carries no such unconditional refusal — it is Tier A
// for the moments path (Table F) — so it reaches this same gate instead of
// skipping it. Finiteness is the separate refusal above, already decided for
// every kind before this runs.
func RequireFullFreeformRange(tStart, tEnd float64, what string) error {
	if (tStart == 0 && tEnd == 1) || (tStart == 1 && tEnd == 0) {
		return nil
	}
	return fmt.Errorf(
		`%w: a %s must span its full domain; a trimmed free-form range is never recordable (range [%v, %v])`,
		decaderr.ErrUnsupported, what, tStart, tEnd,
	)
}
