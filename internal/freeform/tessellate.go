package freeform

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// ErrTooManyChords refuses a chord tolerance finer than the mesh cap.
var ErrTooManyChords = fmt.Errorf(`%w: the chord tolerance asks for more than %d chords on one curve`, decaderr.ErrUnsupported, MaxChordsPerWalk)

// MaxChordsPerWalk caps how finely one boundary curve may be chorded. The
// ceiling is set by the cap triangulator, whose ear clipping is quadratic in
// the boundary samples: 2¹⁴ chords keeps the worst cap under a second while
// still admitting sub-micrometre tolerances on any real part (a 10 mm-radius
// circle at the cap carries a sagitta under 2e-7 mm).
//
// The cap is per-curve, and there is deliberately no total across a profile's
// curves. It bounds what ONE curve can ask of the quadratic cap triangulator,
// which is why ErrTooManyChords reports "more than %d chords on one curve". A
// profile with many loops is proportionally more work because the caller
// modelled proportionally more geometry, and docs/interference-design.md §7
// answers large work with cancellation rather than a work cap: the read-only
// path polls its context at least once per proofbound.WorkPollInterval candidate
// operations, so chordLoop, bridgeHole and earClip all abandon a large profile
// promptly. A total cap would instead refuse a profile this evaluator can
// build correctly.
const MaxChordsPerWalk = 1 << 14
