package loopchain

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Scene contains the sketch handles and exact document values for one loop
// under one drive, on one side of the plane. Zero certifies its starting pose;
// the chain then continues from that enclosure.
type Scene struct {
	Sketch       *sketch.Sketch
	Driver       sketch.Dimension
	Driven       []sketch.Dimension // per dependent: angle or slide distance
	Angular      []bool             // per dependent: read modulo a turn
	Anchored     []bool             // per dependent: read from its anchor
	Signs        []int              // per dependent: axis sense in the scene
	Options      []sketch.EncloseOption
	Pins         []Pin
	ZeroReadings []sketch.Interval // per dependent: reading at the zero pose
	// Offset is the driver's scene reading at the zero pose. Its target is
	// Offset + |q| (docs/linkage-check-design.md §15.2).
	Offset proofbound.RatInterval
	// Side identifies the scene plane, DriverLink the driver's position in
	// Linkage.Links(), and SlideDriver whether its value is a length.
	Side        int
	DriverLink  int
	SlideDriver bool
}

// Pin pairs a scene point with its world position and an enclosure of its
// exact plane position in the document.
type Pin struct {
	Point *sketch.Point
	At    r3.Vec
	U, V  proofbound.RatInterval
}

// Ask holds one certified enclosure or the refusal continued from it.
type Ask struct {
	Enclosure *sketch.Enclosure
	Turns     []int64
	Err       error
}

// ZeroRange bounds the driver's scene reading at the zero pose.
func ZeroRange(offset proofbound.RatInterval) (float64, float64) {
	return proofbound.RatFloatDown(offset.Lo), proofbound.RatFloatUp(offset.Hi)
}

// Zero asks E0 and rejects a scene whose pin boxes miss the document's pins.
// unsupported is the caller's public unsupported-error sentinel.
func Zero(ctx context.Context, sc Scene, unsupported error) (Ask, []sketch.Interval, error) {
	lo, hi := ZeroRange(sc.Offset)
	enc, err := sc.Sketch.Enclose(ctx, sc.Driver, lo, hi, sc.Options...)
	if err != nil {
		if cerr := ctx.Err(); cerr != nil {
			return Ask{}, nil, cerr
		}
		if invariant(err) {
			return Ask{}, nil, err
		}
		return Ask{}, nil, fmt.Errorf(`%w: the loop cannot be enclosed at its zero pose: %w`, unsupported, err)
	}
	for _, pin := range sc.Pins {
		x, y, ok := enc.PointBox(pin.Point)
		if !ok || !intervalHolds(x, pin.U) || !intervalHolds(y, pin.V) {
			return Ask{}, nil, fmt.Errorf(`%w: the loop's zero-pose enclosure does not hold the document's pin at %v, in the loop's plane (%v, %v), so it does not describe the document's mechanism`,
				unsupported, pin.At, pin.U.Lo.FloatString(6), pin.V.Lo.FloatString(6))
		}
	}
	readings := make([]sketch.Interval, 0, len(sc.Driven))
	for _, d := range sc.Driven {
		r, ok := enc.Driven(d)
		if !ok {
			return Ask{}, nil, fmt.Errorf(`%w: the loop's zero-pose enclosure reads no value for a dependent joint`, unsupported)
		}
		readings = append(readings, r)
	}
	if err := sc.anchorsAhead(enc); err != nil {
		return Ask{}, nil, fmt.Errorf(`%w: %w`, unsupported, err)
	}
	return Ask{Enclosure: enc, Turns: make([]int64, len(sc.Driven))}, readings, nil
}

// Continue asks the next range on the same scene. A refusal stays in the ask;
// only cancellation or an invariant failure returns an error.
func Continue(ctx context.Context, sc Scene, lo, hi float64, pred Ask) (Ask, error) {
	if pred.Err != nil {
		return Ask{Err: pred.Err}, nil //nolint:nilerr // predecessor refusals are recorded asks
	}
	if lo > hi {
		return Ask{Err: fmt.Errorf(`%w: the driver range [%v, %v] is empty`, sketch.ErrNotCertified, lo, hi)}, nil
	}
	opts := append(slices.Clone(sc.Options), sketch.WithContinuation(pred.Enclosure))
	if budget, ok := PieceBudget(lo, hi, sc.SlideDriver); ok {
		opts = append(opts, sketch.WithMaxPieces(budget))
	}
	enc, err := sc.Sketch.Enclose(ctx, sc.Driver, lo, hi, opts...)
	if err != nil {
		if cerr := ctx.Err(); cerr != nil {
			return Ask{}, cerr
		}
		if invariant(err) {
			return Ask{}, err
		}
		var fold *sketch.FoldError
		if errors.As(err, &fold) {
			err = sc.foldRefusal(fold, err)
		}
		return Ask{Err: err}, nil
	}
	if err := sc.anchorsAhead(enc); err != nil {
		return Ask{Err: err}, nil //nolint:nilerr // refused asks are recorded
	}
	ask := Ask{Enclosure: enc, Turns: make([]int64, len(sc.Driven))}
	prevPieces, pieces := pred.Enclosure.Pieces(), enc.Pieces()
	last, first := prevPieces[len(prevPieces)-1], pieces[0]
	for j, d := range sc.Driven {
		l, okL := last.Driven(d)
		f, okF := first.Driven(d)
		if !okL || !okF {
			return Ask{Err: fmt.Errorf(`%w: an enclosure reads no value for a dependent joint`, sketch.ErrNotCertified)}, nil
		}
		m := 0.0
		if sc.Angular[j] {
			m = math.Round(((l.Lo+l.Hi)/2 - (f.Lo+f.Hi)/2) / (2 * math.Pi))
		}
		if math.IsNaN(m) || math.Abs(m) > 1<<40 {
			return Ask{Err: fmt.Errorf(`%w: a dependent reading cannot be continued`, sketch.ErrNotCertified)}, nil
		}
		shift := int64(m)
		if !turnsOverlap(f, shift, l) {
			return Ask{Err: fmt.Errorf(`%w: a dependent reading does not continue its predecessor's by any whole turn`, sketch.ErrNotCertified)}, nil
		}
		ask.Turns[j] = pred.Turns[j] + shift
	}
	return ask, nil
}

// Value reads dependent j's displacement from the zero pose in joint sense.
func Value(sc Scene, ask Ask, j int) (motionbound.MotionParam, motionbound.MotionParam) {
	iv, _ := ask.Enclosure.Driven(sc.Driven[j])
	r0 := sc.ZeroReadings[j]
	lo := new(big.Rat).Sub(proofarith.FloatRat(iv.Lo), proofarith.FloatRat(r0.Hi))
	hi := new(big.Rat).Sub(proofarith.FloatRat(iv.Hi), proofarith.FloatRat(r0.Lo))
	turn := big.NewRat(ask.Turns[j], 1)
	if sc.Signs[j] < 0 {
		lo, hi = hi.Neg(hi), lo.Neg(lo)
		turn.Neg(turn)
	}
	return motionbound.MotionParam{Turn: turn, Base: lo}, motionbound.MotionParam{Turn: new(big.Rat).Set(turn), Base: hi}
}

// anchorsAhead rejects an anchored slide whose certified reading can reach its
// anchor. Only a positive reading proves displacement is reading minus E0.
func (sc Scene) anchorsAhead(enc *sketch.Enclosure) error {
	for j, d := range sc.Driven {
		if !sc.Anchored[j] {
			continue
		}
		if r, ok := enc.Driven(d); !ok || !(r.Lo > 0) {
			return fmt.Errorf(`%w: a slide's reading from its anchor is not proven positive`, sketch.ErrNotCertified)
		}
	}
	return nil
}

// intervalHolds only rejects: overlap alone does not place a pin in its box.
func intervalHolds(iv sketch.Interval, x proofbound.RatInterval) bool {
	lo, hi := proofarith.FloatRat(iv.Lo), proofarith.FloatRat(iv.Hi)
	return lo != nil && hi != nil && lo.Cmp(x.Lo) <= 0 && x.Hi.Cmp(hi) <= 0
}

// invariant identifies sketch refusals that cannot arise from a valid scene.
func invariant(err error) bool {
	return errors.Is(err, sketch.ErrUncertifiedConstraint) || errors.Is(err, sketch.ErrForeignHandle) || errors.Is(err, sketch.ErrNonFiniteGeometry)
}

// PiecesPerTurn is the ask budget per whole angular turn.
const PiecesPerTurn = 4096

// PieceBudget returns a larger budget for angular ranges beyond one turn.
func PieceBudget(lo, hi float64, slideDriver bool) (int, bool) {
	turns := math.Ceil((hi - lo) / (2 * math.Pi))
	if slideDriver || !(turns > 1) {
		return 0, false
	}
	return PiecesPerTurn * int(min(turns, 1<<20)), true
}

// foldError restates sketch's fold proof in the driver's joint value and
// unwraps to the original sketch error.
type foldError struct {
	link   int
	lo, hi *big.Rat
	unit   string
	cause  error
}

func (e *foldError) Error() string {
	limit := linkagebound.DecimalUp(e.hi)
	if e.hi.Sign() <= 0 {
		limit = linkagebound.DecimalDown(e.lo)
	}
	return fmt.Sprintf(`the mechanism folds: on the zero pose's branch link %d's joint turns back at a value in [%s, %s] %s, and driven from the zero pose without reversing it never passes %s %s`,
		e.link, linkagebound.DecimalDown(e.lo), linkagebound.DecimalUp(e.hi), e.unit, limit, e.unit)
}

func (e *foldError) Unwrap() error { return e.cause }

// foldRefusal subtracts the scene's zero offset and reverses the negative
// side to state the fold in the driver's own joint value.
func (sc Scene) foldRefusal(fold *sketch.FoldError, cause error) error {
	lo, hi := proofarith.FloatRat(fold.Fold.Lo), proofarith.FloatRat(fold.Fold.Hi)
	if lo == nil || hi == nil {
		return cause
	}
	lo.Sub(lo, sc.Offset.Hi)
	hi.Sub(hi, sc.Offset.Lo)
	if sc.Side == 1 {
		lo, hi = hi.Neg(hi), lo.Neg(lo)
	}
	unit := "rad"
	if sc.SlideDriver {
		unit = "mm"
	}
	return &foldError{link: sc.DriverLink, lo: lo, hi: hi, unit: unit, cause: cause}
}

// turnsOverlap rejects a continued angle when its shifted first reading and
// its predecessor's last reading are proven disjoint for every enclosed π.
func turnsOverlap(f sketch.Interval, m int64, l sketch.Interval) bool {
	turn := big.NewRat(m, 1)
	shiftLo := motionbound.ParamLower(motionbound.MotionParam{Turn: turn, Base: proofarith.FloatRat(f.Lo)})
	shiftHi := motionbound.ParamUpper(motionbound.MotionParam{Turn: turn, Base: proofarith.FloatRat(f.Hi)})
	return shiftLo.Cmp(proofarith.FloatRat(l.Hi)) <= 0 && proofarith.FloatRat(l.Lo).Cmp(shiftHi) <= 0
}
