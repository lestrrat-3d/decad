package prismcells

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// MaxArrangementSegments bounds the private sketch arrangement before
// Profiles starts. The pinned arranger densifies each line to one tiny
// segment and each admitted circle or arc to no more than 256. It compares
// every tiny-segment pair in one quadratic pass. Profiles has no context, so
// this cap bounds the longest stretch a cancelled caller must wait through.
// The pass costs about 8.3e-5 ms per segment squared, so this cap bounds
// one arrangement at roughly 1.4 seconds and peak memory at twice the cap
// under 16 MB. It admits a rectangular plate with fourteen circular holes
// and one circular tool (agent-docs/prism-boolean-design.md §10).
const MaxArrangementSegments = 4096

// RegionsWithinWorkCap counts the densified segments over every region a
// private scene will hold. A prism group's lumps are separate regions.
func RegionsWithinWorkCap(budget *proofbound.WorkBudget, profiles ...momentinput.Profile) (int, bool, error) {
	return RegionsWithinSegmentCap(budget, MaxArrangementSegments, profiles...)
}

// arcTurnSegments is the pinned arranger's densification of one full turn:
// sketch's adaptive default, geom/arrange.go, which splits an arc into
// ceil(256·sweep/2π) pieces (at least 2) and a whole circle into 256.
const arcTurnSegments = 256

// sweepSlack covers the float reading of an arc's sweep below: two atan2
// evaluations, a remainder and a difference, each within a few ulps of 2π.
const sweepSlack = 0x1p-40

// CircularSegmentCount is an upper bound on the pieces the pinned arranger
// densifies one recorded circle or arc into
// (agent-docs/carrier-record-design.md §4). A whole circle counts the
// arranger's own 256. An ArcSeg's turn is sketch's wrapSweep of its
// recorded centre, start and end, computed with math.Mod as sketch computes
// it, multiplied by a narrowed range; a narrowed CircleSeg's turn is its
// range. The count is min(257, max(2, ⌈256·(turn + 2⁻⁴⁰)⌉ + 1)), and 257 for
// a turn that is not finite, not positive or above one.
func CircularSegmentCount(seg momentinput.CurveSegment) int {
	turn := 1.0
	switch s := seg.(type) {
	case momentinput.CircleSeg:
		if WholeSegmentRange(s.TStart, s.TEnd) {
			// sketch densifies a closed circle to exactly one turn's pieces,
			// with no float sweep to read.
			return arcTurnSegments
		}
		turn = math.Abs(s.TEnd - s.TStart)
	case momentinput.ArcSeg:
		turn = arcSweep(s.Center, s.Start, s.End) / (2 * math.Pi)
		if !WholeSegmentRange(s.TStart, s.TEnd) {
			turn *= math.Abs(s.TEnd - s.TStart)
		}
	}
	if math.IsNaN(turn) || turn <= 0 || turn > 1 {
		return arcTurnSegments + 1
	}
	n := int(math.Ceil(arcTurnSegments*(turn+sweepSlack))) + 1
	return min(arcTurnSegments+1, max(2, n))
}

// arcSweep is sketch's wrapSweep (geom/geom.go) of the bearings of start and
// end about centre, a value in (0, 2π].
func arcSweep(center, start, end momentinput.Point2) float64 {
	a0 := math.Atan2(start.V-center.V, start.U-center.U)
	a1 := math.Atan2(end.V-center.V, end.U-center.U)
	d := math.Mod(a1-a0, 2*math.Pi)
	if d <= 0 {
		d += 2 * math.Pi
	}
	return d
}

// SketchSceneSegments is the arranger's own densified segment count of a
// scene: sketch's formula over each created entity's floats (a line one
// segment, a circle 256, an arc max(2, ⌈256·sweep/2π⌉)). It is the
// authoritative count the screen RegionsWithinWorkCap bounds.
func SketchSceneSegments(s *sketch.Sketch) int {
	total := 0
	for _, ent := range s.Entities() {
		switch e := ent.(type) {
		case *sketch.Line:
			total++
		case *sketch.Circle:
			total += arcTurnSegments
		case *sketch.Arc:
			total += max(2, int(math.Ceil(arcTurnSegments*e.Sweep()/(2*math.Pi))))
		}
	}
	return total
}

// CheckSceneWithinCap returns WithWorkLimit(ErrUnsupported…) when the scene's
// authoritative count exceeds the arrangement cap. A caller whose over-cap
// answer is a silent miss maps the error through errors.As to
// *WorkLimitError.
func CheckSceneWithinCap(s *sketch.Sketch) error {
	n := SketchSceneSegments(s)
	if n <= MaxArrangementSegments {
		return nil
	}
	return WithWorkLimit(fmt.Errorf(
		`%w: the scene's arranged entities densify to %d arranger segments against this evaluator's cap of %d; combine the sections into one profile instead of applying this op once per feature`,
		decaderr.ErrUnsupported, n, MaxArrangementSegments), n)
}
